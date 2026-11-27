#include "pinky_soccer_bt/bt_nodes.hpp"

#include <cmath>
#include <atomic>
#include <set>
#include <behaviortree_cpp/contrib/json.hpp>

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
{
  contact_sub_ = node_.lock()->create_subscription<std_msgs::msg::String>("world_model/ball_contact", 10,
    [this](const std_msgs::msg::String& msg) {
      try {
        const auto d = nlohmann::json::parse(msg.data);
        contact_stamp_ = d.at("stamp").get<double>();
        contact_present_ = d.at("present").get<bool>();
      } catch(const std::exception&) { contact_present_ = false; }
    });
}

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
  config().blackboard->set("@world_stamp", seen_->header.stamp.sec + 1e-9 * seen_->header.stamp.nanosec);
  const double world_stamp = seen_->header.stamp.sec + 1e-9 * seen_->header.stamp.nanosec;
  config().blackboard->set("@ball_contact", contact_present_ && world_stamp-contact_stamp_ >= -.05 &&
                           world_stamp-contact_stamp_ < .35);
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
{ policy_pub_ = node_->create_publisher<std_msgs::msg::String>("motion/policy", 10); }

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
  std::string policy;
  if(goal.action == "GoToPose" && config().blackboard->get("@motion_policy", policy))
  {
    std_msgs::msg::String message; message.data = policy; policy_pub_->publish(message);
  }
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

PortsList AttackBall::providedPorts()
{
  return {InputPort<TrackedObject>("ball"), InputPort<TrackedObject>("pose"),
          InputPort<Point>("opp_goal"), InputPort<bool>("planner_approach", false, "Let path planner choose the detour"), InputPort<double>("blind_distance", 0.22, "Max unobserved push distance [m]"),
          InputPort<double>("blind_seconds", 1.25, "Max unobserved push time [s]"), InputPort<double>("backup_distance", 0.16, "Camera reacquisition backup [m]")};
}

NodeStatus AttackBall::update()
{
  const auto self = input<TrackedObject>("pose");
  const auto p = toPose(self);
  const auto b = input<TrackedObject>("ball");
  const auto ball = toVec(b);
  const auto goal = toVec(input<Point>("opp_goal"));
  double now = 0.;
  (void)config().blackboard->get("@world_stamp", now);
  const bool known = ballKnown(b.state == TrackedObject::LOST, cov(b), 0.20);
  const auto emit = [&](const std::string& action, const Pose2& target) {
    config().blackboard->set(kGoalKey, makeGoal("ATTACK", action, BehaviorGoal::REACTIVE, target, 0.0));
  };
  if(known)
  {
    const double old_distance = std::hypot(goal.x-ball_progress_.x,goal.y-ball_progress_.y);
    const double distance = std::hypot(goal.x-ball.x,goal.y-ball.y);
    if(progress_time_ < 0. || old_distance-distance > .06)
    { ball_progress_ = ball; progress_time_ = now; }
    if((stage_ == 0 || (stage_ == 1 && now-seen_ < input<double>("blind_seconds"))) &&
       now-progress_time_ > 8. &&
       std::hypot(p.x-ball.x,p.y-ball.y) < .4)
    {
      const double dx=goal.x-ball.x,dy=goal.y-ball.y,n=std::max(.01,std::hypot(dx,dy));
      const double ux=dx/n,uy=dy/n;
      const double side = ux*(p.y-ball.y)-uy*(p.x-ball.x) >= 0. ? -1. : 1.;
      flank_ = {ball.x-.12*ux-.18*side*uy, ball.y-.12*uy+.18*side*ux,std::atan2(uy,ux)};
      flank_until_ = now+2.5; stage_ = 3;
    }
  }
  if(stage_ == 3)
  {
    if(now < flank_until_ && std::hypot(p.x-flank_.x,p.y-flank_.y) > .04)
    { emit("GoBehindBall",flank_); return NodeStatus::RUNNING; }
    stage_ = 0; progress_time_ = now; ball_progress_ = ball;
  }
  if(stage_ == 2)
  {
    if(std::hypot(p.x-backup_.x, p.y-backup_.y) > 0.035 && now-backup_started_ < 2.0)
    {
      emit("Backoff", backup_);
      return NodeStatus::RUNNING;
    }
    stage_ = 0;
  }
  bool contact = false;
  (void)config().blackboard->get("@ball_contact", contact);
  if(stage_ == 1)
  {
    if(contact) { anchor_ = p; seen_ = now; }
    if(b.state == TrackedObject::OBSERVED)
    {
      anchor_ = p;
      seen_ = now;
      const double lateral = std::abs(-std::sin(p.theta)*(ball.x-p.x)+std::cos(p.theta)*(ball.y-p.y));
      if(lateral > 0.10 && !contact) stage_ = 0;
    }
    if(stage_ == 1)
    {
      if(now-seen_ > input<double>("blind_seconds") ||
         std::hypot(p.x-anchor_.x, p.y-anchor_.y) > input<double>("blind_distance"))
      {
        const double d = input<double>("backup_distance");
        backup_ = {p.x-d*std::cos(p.theta), p.y-d*std::sin(p.theta), p.theta};
        stage_ = 2; backup_started_ = now;
        emit("Backoff", backup_);
      }
      else
      {
        emit("PushBall", {goal.x, goal.y, std::atan2(goal.y-p.y, goal.x-p.x)});
      }
      return NodeStatus::RUNNING;
    }
  }
  if(!known)
  {
    emit("SpinInPlace", {p.x, p.y, wrapAngle(p.theta+1.0)});
  }
  else if(isBehindBall(p, ball, goal, 0.05, 0.25, 0.30))
  {
    stage_ = 1; anchor_ = p; seen_ = now;
    emit("PushBall", {goal.x, goal.y, std::atan2(goal.y-p.y, goal.x-p.x)});
  }
  else if(isBehindBall(p,ball,goal,.05,3.141592653589793,.30))
  {
    // Position is already behind the ball: adjust yaw here, do not drive back
    // to a new approach point and orbit the ball merely to fix heading.
    emit("AlignToBall", {p.x,p.y,std::atan2(goal.y-ball.y,goal.x-ball.x)});
  }
  else
  {
    // Shorten the behind offset near our end wall. A fixed 15 cm offset
    // can make the side waypoint reachable but its behind condition impossible.
    const double offset = ball.x*goal.x < 0. ?
        std::min(.15,std::max(.055,std::abs(goal.x)-std::abs(ball.x)-.02)) : .15;
    const double norm = std::max(.01,std::hypot(goal.x-ball.x,goal.y-ball.y));
    const double ux=(goal.x-ball.x)/norm,uy=(goal.y-ball.y)/norm;
    const double along=(p.x-ball.x)*ux+(p.y-ball.y)*uy;
    const double cross=(p.y-ball.y)*ux-(p.x-ball.x)*uy;
    // Separate entry/exit thresholds and retain the orbit side through small
    // observation noise. Reversing the detour every tick prevents real progress.
    if(approach_orbit_ && along < -.6*offset) approach_orbit_ = false;
    else if(!approach_orbit_ && along > -.25*offset) approach_orbit_ = true;
    if(approach_side_ == 0 || cross*approach_side_ < -.08)
      approach_side_ = cross >= 0. ? 1 : -1;
    auto approach = behindBallPose(ball,goal,offset);
    if(approach_orbit_ && !input<bool>("planner_approach"))
    { approach.x -= .20*approach_side_*uy; approach.y += .20*approach_side_*ux; }
    if(std::abs(ball.y)>.25 && (approach.y-ball.y)*ball.y>0.)
      approach.y = 2*ball.y-approach.y;
    emit("GoBehindBall", approach);
  }
  return NodeStatus::RUNNING;
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
          InputPort<double>("theta", 0.0, "Final heading [rad, map]"),
          // Optional NMPC collision constraints; all false = the unchanged GoToPose.
          InputPort<bool>("avoid_walls", false, "Keep the footprint off walls and goals"),
          InputPort<bool>("avoid_opponents", false, "Keep clear of tracked opponents"),
          InputPort<bool>("avoid_ball", false, "Keep clear of the ball"),
          InputPort<std::string>("completion_mode", "DEFAULT", "DEFAULT/LEGACY/STRICT_STOP/PRACTICAL_STOP/PASS_THROUGH/HANDOFF"),
          InputPort<double>("task_position", 0.02, "Task position acceptance [m]"),
          InputPort<double>("task_heading", 0.05235987755982989, "Task heading acceptance [rad]"),
          InputPort<double>("task_dwell", 0.1, "PRACTICAL_STOP standstill duration [s]"),
          InputPort<bool>("next_ready", false, "Successor contract available; controller must prepare its safe trajectory"),
          InputPort<double>("next_x", 0.0, "Successor map X [m]"),
          InputPort<double>("next_y", 0.0, "Successor map Y [m]"),
          InputPort<double>("next_theta", 0.0, "Successor map heading [rad]")};
}

NodeStatus GoToPose::onStart()
{
  static std::atomic<uint64_t> next_id{
      static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())};
  goal_id_ = ++next_id;
  target_ = Pose2{input<double>("x"), input<double>("y"), input<double>("theta")};
  // Fixed for this goal_id: a different option set is a new request.
  avoid_walls_ = input<bool>("avoid_walls");
  avoid_opponents_ = input<bool>("avoid_opponents");
  avoid_ball_ = input<bool>("avoid_ball");
  completion_mode_ = input<std::string>("completion_mode");
  const std::set<std::string> modes{"DEFAULT", "LEGACY", "STRICT_STOP", "PRACTICAL_STOP", "PASS_THROUGH", "HANDOFF"};
  if(!modes.count(completion_mode_)) throw BT::RuntimeError("invalid completion_mode");
  nlohmann::json policy{{"goal_id", std::to_string(goal_id_)}, {"mode", completion_mode_},
      {"position", input<double>("task_position")}, {"heading", input<double>("task_heading")},
      {"dwell", input<double>("task_dwell")}, {"next_ready", input<bool>("next_ready")}};
  if(input<bool>("next_ready")) policy["next_target"] =
      {input<double>("next_x"), input<double>("next_y"), input<double>("next_theta")};
  config().blackboard->set("@motion_policy", policy.dump());
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
  goal.avoid_walls = avoid_walls_;
  goal.avoid_opponents = avoid_opponents_;
  goal.avoid_ball = avoid_ball_;
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
      if(report.result.reason != "handoff_ready" ||
         (completion_mode_ != "PASS_THROUGH" && completion_mode_ != "HANDOFF"))
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
  factory.registerNodeType<AttackBall>("AttackBall");
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
