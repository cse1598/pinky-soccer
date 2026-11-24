#include "pinky_soccer_bt/bt_nodes.hpp"

#include <cmath>
#include <atomic>

namespace pinky_soccer_bt
{

using BT::InputPort;
using BT::NodeStatus;
using BT::OutputPort;
using BT::PortsList;

BehaviorGoal makeGoal(const std::string& branch, const std::string& action, uint8_t mode,
                      const Pose2& target, double max_speed)
{
  BehaviorGoal g;
  g.header.frame_id = "map";
  g.branch = branch;
  g.action = action;
  g.mode = mode;
  g.target.x = target.x;
  g.target.y = target.y;
  g.target.theta = target.theta;
  g.max_speed = static_cast<float>(max_speed);
  g.planned_path.header.frame_id = "map";
  return g;
}

Pose2 toPose(const TrackedObject& o)
{
  return {o.pose.x, o.pose.y, o.pose.theta};
}

Vec2 toVec(const TrackedObject& o)
{
  return {o.pose.x, o.pose.y};
}

Vec2 toVec(const Point& p)
{
  return {p.x, p.y};
}

namespace
{
std::array<double, 9> cov(const TrackedObject& o)
{
  std::array<double, 9> c{};
  std::copy(o.covariance.begin(), o.covariance.end(), c.begin());
  return c;
}

// Self pose written by ReadWorldModel into the root blackboard.
Pose2 rootSelf(const BT::Blackboard::Ptr& bb)
{
  return toPose(bb->get<TrackedObject>("@pose"));
}

constexpr double kReachTol = 0.05;     // [m] position tolerance for "arrived"
constexpr double kHeadingTol = 0.15;   // [rad]
}  // namespace

// ---------------------------------------------------------------- ReadWorldModel
ReadWorldModel::ReadWorldModel(const std::string& name, const BT::NodeConfig& conf,
                               const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<WorldModel>(name, conf, params)
{}

PortsList ReadWorldModel::providedPorts()
{
  return providedBasicPorts({
      InputPort<double>("stale_sec", 0.5, "Max age of the latest message [s, steady clock]"),
      OutputPort<bool>("playing"),
      OutputPort<TrackedObject>("pose"),
      OutputPort<TrackedObject>("ball"),
      OutputPort<Opponents>("opponents"),
      OutputPort<Point>("own_goal"),
      OutputPort<Point>("opp_goal"),
  });
}

NodeStatus ReadWorldModel::onTick(const std::shared_ptr<WorldModel>& last_msg)
{
  const auto now = std::chrono::steady_clock::now();
  if(last_msg && last_msg != seen_)
  {
    seen_ = last_msg;
    received_ = now;
  }
  const double stale = getInput<double>("stale_sec").value_or(0.5);
  const double age = std::chrono::duration<double>(now - received_).count();
  if(!seen_ || age > stale)
  {
    config().blackboard->set(kGoalKey, makeGoal("STOP", seen_ ? "WorldModelStale" :
                                                                "NoWorldModel",
                                                 BehaviorGoal::STOP, Pose2{}, 0.0));
    return NodeStatus::FAILURE;
  }
  setOutput("playing", static_cast<bool>(seen_->playing));
  setOutput("pose", seen_->self);
  setOutput("ball", seen_->ball);
  setOutput("opponents", Opponents(seen_->opponents.begin(), seen_->opponents.end()));
  setOutput("own_goal", seen_->own_goal);
  setOutput("opp_goal", seen_->opp_goal);
  return NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------- ReadGameState
ReadGameState::ReadGameState(const std::string& name, const BT::NodeConfig& conf,
                             const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{}

PortsList ReadGameState::providedPorts()
{
  return providedBasicPorts({OutputPort<bool>("kickoff")});
}

NodeStatus ReadGameState::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg)
{
  if(last_msg)
  {
    kickoff_ = last_msg->data;
  }
  setOutput("kickoff", kickoff_);
  return NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------- PublishBehaviorGoal
PublishBehaviorGoal::PublishBehaviorGoal(const std::string& name, const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosTopicPubNode<BehaviorGoal>(name, conf, params)
{}

PortsList PublishBehaviorGoal::providedPorts()
{
  return providedBasicPorts({});
}

bool PublishBehaviorGoal::setMessage(BehaviorGoal& msg)
{
  BehaviorGoal goal;
  if(!config().blackboard->get(kGoalKey, goal))
  {
    goal = makeGoal("STOP", "NoGoal", BehaviorGoal::STOP, Pose2{}, 0.0);
  }
  msg = goal;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  return true;
}

// ---------------------------------------------------------------- conditions
PortsList IsPlaying::providedPorts()
{
  return {InputPort<bool>("playing")};
}

NodeStatus IsPlaying::tick()
{
  return getInput<bool>("playing").value_or(false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

PortsList KickoffRequested::providedPorts()
{
  return {InputPort<bool>("kickoff")};
}

NodeStatus KickoffRequested::tick()
{
  return getInput<bool>("kickoff").value_or(false) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

PortsList BallKnown::providedPorts()
{
  return {InputPort<TrackedObject>("ball"),
          InputPort<double>("max_sigma", 0.30, "Max position std-dev [m]")};
}

NodeStatus BallKnown::tick()
{
  auto ball = getInput<TrackedObject>("ball");
  if(!ball)
  {
    return NodeStatus::FAILURE;
  }
  const double max_sigma = getInput<double>("max_sigma").value_or(0.30);
  return ballKnown(ball->state == TrackedObject::LOST, cov(ball.value()), max_sigma) ?
             NodeStatus::SUCCESS :
             NodeStatus::FAILURE;
}

PortsList BallInOwnHalf::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<Point>("own_goal"),
          InputPort<double>("margin", 0.30, "Own half extends this far past the centre [m]")};
}

NodeStatus BallInOwnHalf::tick()
{
  auto ball = getInput<TrackedObject>("ball");
  auto own = getInput<Point>("own_goal");
  if(!ball || !own)
  {
    return NodeStatus::FAILURE;
  }
  return ballInOwnHalf(toVec(ball.value()), toVec(own.value()),
                       getInput<double>("margin").value_or(0.30)) ?
             NodeStatus::SUCCESS :
             NodeStatus::FAILURE;
}

PortsList OpponentCloserToBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<Opponents>("opponents"),
          InputPort<TrackedObject>("pose")};
}

NodeStatus OpponentCloserToBall::tick()
{
  auto ball = getInput<TrackedObject>("ball");
  auto opps = getInput<Opponents>("opponents");
  auto pose = getInput<TrackedObject>("pose");
  if(!ball || !opps || !pose)
  {
    return NodeStatus::FAILURE;
  }
  std::vector<Vec2> visible;
  for(const auto& o : opps.value())
  {
    if(o.state != TrackedObject::LOST)
    {
      visible.push_back(toVec(o));
    }
  }
  return opponentCloserToBall(toVec(ball.value()), toPose(pose.value()), visible) ?
             NodeStatus::SUCCESS :
             NodeStatus::FAILURE;
}

PortsList IsBehindBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose"),
          InputPort<Point>("opp_goal"),
          InputPort<double>("tol_dist", 0.08, "Max lateral distance from the shot line [m]"),
          InputPort<double>("tol_angle", 0.35, "Max heading error to the shot line [rad]"),
          InputPort<double>("max_behind", 0.30, "Max distance behind the ball [m]")};
}

NodeStatus IsBehindBall::tick()
{
  auto ball = getInput<TrackedObject>("ball");
  auto pose = getInput<TrackedObject>("pose");
  auto goal = getInput<Point>("opp_goal");
  if(!ball || !pose || !goal)
  {
    return NodeStatus::FAILURE;
  }
  return isBehindBall(toPose(pose.value()), toVec(ball.value()), toVec(goal.value()),
                      getInput<double>("tol_dist").value_or(0.08),
                      getInput<double>("tol_angle").value_or(0.35),
                      getInput<double>("max_behind").value_or(0.30)) ?
             NodeStatus::SUCCESS :
             NodeStatus::FAILURE;
}

PortsList NotFacingOwnGoal::providedPorts()
{
  return {InputPort<TrackedObject>("pose"), InputPort<Point>("own_goal"),
          InputPort<double>("min_angle", 1.57, "Min angle between heading and own goal [rad]")};
}

NodeStatus NotFacingOwnGoal::tick()
{
  auto pose = getInput<TrackedObject>("pose");
  auto own = getInput<Point>("own_goal");
  if(!pose || !own)
  {
    return NodeStatus::FAILURE;
  }
  return notFacingOwnGoal(toPose(pose.value()), toVec(own.value()),
                          getInput<double>("min_angle").value_or(1.57)) ?
             NodeStatus::SUCCESS :
             NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- actions
GoalAction::GoalAction(const std::string& name, const BT::NodeConfig& conf, std::string branch)
  : BT::StatefulActionNode(name, conf), branch_(std::move(branch))
{}

void GoalAction::setGoal(uint8_t mode, const Pose2& target, double max_speed,
                         const std::vector<Pose2>& path)
{
  auto goal = makeGoal(branch_, registrationName(), mode, target, max_speed);
  for(const auto& p : path)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation.z = std::sin(0.5 * p.theta);
    pose.pose.orientation.w = std::cos(0.5 * p.theta);
    goal.planned_path.poses.push_back(pose);
  }
  config().blackboard->set(kGoalKey, goal);
}

NodeStatus StopRobot::update()
{
  setGoal(BehaviorGoal::STOP, Pose2{}, 0.0);
  return NodeStatus::SUCCESS;
}

PortsList GoBehindBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose"),
          InputPort<Point>("opp_goal"),
          InputPort<double>("offset", 0.15, "Distance behind the ball [m]"),
          InputPort<double>("clearance", 0.20, "Side detour distance from the ball [m]")};
}

NodeStatus GoBehindBall::update()
{
  const auto pose = toPose(input<TrackedObject>("pose"));
  const auto ball = toVec(input<TrackedObject>("ball"));
  const auto goal = toVec(input<Point>("opp_goal"));
  const double offset = input<double>("offset");
  const auto target = approachPose(pose, ball, goal, offset, input<double>("clearance"));
  const auto behind = behindBallPose(ball, goal, offset);
  // Detour: the side waypoint comes first, then the pose behind the ball (planned_path).
  std::vector<Pose2> path{target};
  if(std::hypot(target.x - behind.x, target.y - behind.y) > 1e-6)
  {
    path.push_back(behind);
  }
  setGoal(BehaviorGoal::REACTIVE, target, 0.0, path);
  return NodeStatus::RUNNING;   // IsBehindBall takes over once in position
}

PortsList PushBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<Point>("opp_goal"),
          InputPort<double>("speed", 0.25, "Max push speed [m/s]"),
          InputPort<double>("beyond", 0.10, "Target this far past the goal mouth [m]")};
}

NodeStatus PushBall::update()
{
  const auto target = pushTarget(toVec(input<TrackedObject>("ball")),
                                 toVec(input<Point>("opp_goal")), input<double>("beyond"));
  setGoal(BehaviorGoal::REACTIVE, target, input<double>("speed"));
  return NodeStatus::RUNNING;
}

PortsList GoToBlockPoint::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose"),
          InputPort<Point>("own_goal"),
          InputPort<double>("ratio", 0.35, "Fraction of own goal -> ball segment")};
}

NodeStatus GoToBlockPoint::update()
{
  const auto pose = toPose(input<TrackedObject>("pose"));
  const auto target = blockPose(toVec(input<TrackedObject>("ball")),
                                toVec(input<Point>("own_goal")), input<double>("ratio"));
  setGoal(BehaviorGoal::NAVIGATE, target, 0.0);
  // The block point moves with the ball; hysteresis avoids flipping with FaceBall.
  in_place_ = reachedPosition(pose, target, in_place_ ? 2.0 * kReachTol : kReachTol);
  return in_place_ ? NodeStatus::SUCCESS : NodeStatus::RUNNING;
}

PortsList FaceBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose")};
}

NodeStatus FaceBall::update()
{
  const auto pose = toPose(input<TrackedObject>("pose"));
  setGoal(BehaviorGoal::REACTIVE, facePose(pose, toVec(input<TrackedObject>("ball"))), 0.0);
  return NodeStatus::RUNNING;
}

PortsList TurnToLastSeen::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose")};
}

NodeStatus TurnToLastSeen::update()
{
  const auto pose = toPose(input<TrackedObject>("pose"));
  const auto target = facePose(pose, toVec(input<TrackedObject>("ball")));
  setGoal(BehaviorGoal::REACTIVE, target, 0.0);
  // Facing the last estimate without seeing the ball: give up so the search continues.
  return reachedHeading(pose, target.theta, kHeadingTol) ? NodeStatus::FAILURE :
                                                           NodeStatus::RUNNING;
}

PortsList SpinInPlace::providedPorts()
{
  return {InputPort<double>("speed", 0.8, "Signed turn rate hint [rad/s]; sign = direction")};
}

NodeStatus SpinInPlace::update()
{
  const auto pose = rootSelf(config().blackboard);
  const double speed = input<double>("speed");
  // The goal is a pose; keep the heading target one radian ahead so the controller turns
  // continuously. Speed magnitude is passed as max_speed for the controller's angular cap.
  setGoal(BehaviorGoal::REACTIVE,
          Pose2{pose.x, pose.y, wrapAngle(pose.theta + std::copysign(1.0, speed))},
          std::abs(speed));
  return NodeStatus::RUNNING;
}

PortsList GoToPose::providedPorts()
{
  return {InputPort<double>("x"), InputPort<double>("y"),
          InputPort<double>("theta", 0.0, "Final heading [rad, map]")};
}

NodeStatus GoToPose::onStart()
{
  static std::atomic<uint64_t> next_id{
      static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())};
  goal_id_ = ++next_id;
  target_ = Pose2{input<double>("x"), input<double>("y"), input<double>("theta")};
  started_ = std::chrono::steady_clock::now();
  return update();
}

void GoToPose::onHalted()
{
  // Do not overwrite a new action selected by the tree during the same tick.
  BehaviorGoal current;
  if(config().blackboard->get(kGoalKey, current) && current.goal_id == goal_id_)
  {
    setGoal(BehaviorGoal::STOP, Pose2{}, 0.0);
  }
}

NodeStatus GoToPose::update()
{
  auto goal = makeGoal(branch_, registrationName(), BehaviorGoal::NAVIGATE, target_, 0.0);
  goal.goal_id = goal_id_;
  config().blackboard->set(kGoalKey, goal);
  ReceivedMotionResult report;
  const auto now = std::chrono::steady_clock::now();
  const bool matching = config().blackboard->get("@motion_result", report) &&
                        report.result.goal_id == goal_id_ &&
                        now - report.received < std::chrono::milliseconds(700);
  if(matching)
  {
    if(report.result.state == MotionResult::SUCCESS)
    {
      setGoal(BehaviorGoal::STOP, Pose2{}, 0.0);
      return NodeStatus::SUCCESS;
    }
    if(report.result.state == MotionResult::FAILURE)
    {
      setGoal(BehaviorGoal::STOP, Pose2{}, 0.0);
      return NodeStatus::FAILURE;
    }
  }
  else if(now - started_ > std::chrono::seconds(3))
  {
    setGoal(BehaviorGoal::STOP, Pose2{}, 0.0);
    return NodeStatus::FAILURE;
  }
  return NodeStatus::RUNNING;
}

// ---------------------------------------------------------------- registration
void registerSoccerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& world_model,
                         const BT::RosNodeParams& goal)
{
  factory.registerNodeType<ReadWorldModel>("ReadWorldModel", world_model);
  factory.registerNodeType<PublishBehaviorGoal>("PublishBehaviorGoal", goal);
  factory.registerNodeType<ReadGameState>("ReadGameState", world_model);
  factory.registerNodeType<IsPlaying>("IsPlaying");
  factory.registerNodeType<KickoffRequested>("KickoffRequested");
  factory.registerNodeType<BallKnown>("BallKnown");
  factory.registerNodeType<BallInOwnHalf>("BallInOwnHalf");
  factory.registerNodeType<OpponentCloserToBall>("OpponentCloserToBall");
  factory.registerNodeType<IsBehindBall>("IsBehindBall");
  factory.registerNodeType<NotFacingOwnGoal>("NotFacingOwnGoal");
  factory.registerNodeType<StopRobot>("StopRobot");
  factory.registerNodeType<GoBehindBall>("GoBehindBall");
  factory.registerNodeType<PushBall>("PushBall");
  factory.registerNodeType<GoToBlockPoint>("GoToBlockPoint");
  factory.registerNodeType<FaceBall>("FaceBall");
  factory.registerNodeType<TurnToLastSeen>("TurnToLastSeen");
  factory.registerNodeType<SpinInPlace>("SpinInPlace");
  factory.registerNodeType<GoToPose>("GoToPose");
}

void setKickoffPose(const BT::Blackboard::Ptr& blackboard, double x, double y, double theta)
{
  blackboard->set("kickoff_x", x);
  blackboard->set("kickoff_y", y);
  blackboard->set("kickoff_theta", theta);
}

}  // namespace pinky_soccer_bt
