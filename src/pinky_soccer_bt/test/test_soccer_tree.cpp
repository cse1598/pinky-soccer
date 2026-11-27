// Loads the real tree XML, feeds /world_model messages and checks the chosen goal.
#include <gtest/gtest.h>

#include <cstdlib>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "pinky_soccer_bt/bt_nodes.hpp"
#include "pinky_soccer_bt/verify_mode.hpp"

using namespace pinky_soccer_bt;
using pinky_soccer_msgs::msg::TrackedObject;
using pinky_soccer_msgs::msg::WorldModel;

class TreeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("ROS_DOMAIN_ID", "93", 1);   // isolate from running simulators
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("bt_test");
    pub_ = node_->create_publisher<WorldModel>("/world_model", 10);
    registerSoccerNodes(factory_, BT::RosNodeParams(node_, "/world_model"),
                        BT::RosNodeParams(node_, "/behavior_goal_test"));
    factory_.registerBehaviorTreeFromFile(TREE_FILE);
    bb_ = BT::Blackboard::create();
    bb_->set("goal", makeGoal("STOP", "Init", BehaviorGoal::STOP, Pose2{}, 0.0));
    setKickoffPose(bb_, -0.4, 0.1, 0.5);
    tree_ = factory_.createTree("Main", bb_);
  }

  static WorldModel world(double rx, double ry, double rth, double bx, double by,
                          uint8_t ball_state = TrackedObject::OBSERVED, bool playing = true)
  {
    WorldModel w;
    w.header.frame_id = "map";
    w.self.pose.x = rx;
    w.self.pose.y = ry;
    w.self.pose.theta = rth;
    w.ball.pose.x = bx;
    w.ball.pose.y = by;
    w.ball.state = ball_state;
    const double var = ball_state == TrackedObject::LOST ? 1.0 : 0.0004;
    w.ball.covariance[0] = w.ball.covariance[4] = var;
    w.own_goal.x = -1.095;   // our_goal_color: yellow
    w.opp_goal.x = 1.095;
    w.our_goal_color = "yellow";
    w.playing = playing;
    return w;
  }

  // Publish until the tree has consumed this message, then return the goal.
  BehaviorGoal decide(const WorldModel& w) { return decideWith(tree_, w); }

  BehaviorGoal decideWith(BT::Tree& tree, const WorldModel& w,
                          BT::NodeStatus* status = nullptr)
  {
    BehaviorGoal goal;
    for(int i = 0; i < 100; ++i)
    {
      pub_->publish(w);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      tree.tickOnce();
      TrackedObject pose, ball;
      bool playing = false;
      if(bb_->get("pose", pose) && bb_->get("ball", ball) && bb_->get("playing", playing) &&
         pose.pose.x == w.self.pose.x && ball.pose.x == w.ball.pose.x &&
         playing == static_cast<bool>(w.playing))
      {
        const auto result = tree.tickOnce();
        if(status != nullptr)
        {
          *status = result;
        }
        break;
      }
    }
    EXPECT_TRUE(bb_->get("goal", goal));
    return goal;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<WorldModel>::SharedPtr pub_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;
  BT::Tree tree_;
};

TEST_F(TreeTest, NoWorldModelStops)
{
  tree_.tickOnce();
  BehaviorGoal goal;
  EXPECT_TRUE(bb_->get("goal", goal));
  EXPECT_EQ(goal.mode, BehaviorGoal::STOP);
  EXPECT_EQ(goal.action, "NoWorldModel");
}

TEST_F(TreeTest, NotPlayingStops)
{
  const auto g = decide(world(0, 0, 0, 0.5, 0, TrackedObject::OBSERVED, false));
  EXPECT_EQ(g.branch, "STOP");
  EXPECT_EQ(g.action, "StopRobot");
}

TEST_F(TreeTest, BehindBallPushes)
{
  const auto g = decide(world(0.3, 0, 0, 0.5, 0));
  EXPECT_EQ(g.branch, "ATTACK");
  EXPECT_EQ(g.action, "PushBall");
  EXPECT_EQ(g.mode, BehaviorGoal::REACTIVE);
  EXPECT_GT(g.target.x, 1.095);
}

TEST_F(TreeTest, NotBehindGoesBehindBall)
{
  const auto g = decide(world(0.3, 0.4, 0, 0.6, 0));
  EXPECT_EQ(g.action, "GoBehindBall");
}

TEST_F(TreeTest, BallInOwnHalfDefends)
{
  const auto g = decide(world(0, 0, 0, -0.6, 0.2));
  EXPECT_EQ(g.branch, "DEFEND");
  EXPECT_EQ(g.action, "GoToBlockPoint");
  EXPECT_LT(g.target.x, -0.6);   // between ball and own goal
}

TEST_F(TreeTest, LostBallSearches)
{
  const auto g = decide(world(0, 0, 0, 0.5, 0.5, TrackedObject::LOST));
  EXPECT_EQ(g.branch, "SEARCH");
  EXPECT_EQ(g.action, "TurnToLastSeen");
}

TEST_F(TreeTest, PublishTreeSendsGoal)
{
  auto pub_tree = factory_.createTree("PublishGoal", bb_);
  BehaviorGoal received;
  bool got = false;
  auto sub = node_->create_subscription<BehaviorGoal>(
      "/behavior_goal", 10, [&](BehaviorGoal::SharedPtr m) {
        received = *m;
        got = true;
      });
  for(int i = 0; i < 100 && !got; ++i)
  {
    pub_tree.tickOnce();
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(got);
  EXPECT_EQ(received.action, "Init");
  EXPECT_EQ(received.header.frame_id, "map");
}

// ---- verification mode: single behaviours built from the same factory -------------------
TEST_F(TreeTest, VerifyXmlRejectsBadRequests)
{
  using pinky_soccer_bt::buildVerifyXml;
  using pinky_soccer_bt::NodeCall;
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"Nope", {}}, "/world_model"),
               std::invalid_argument);
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"ReadWorldModel", {}}, "/world_model"),
               std::invalid_argument);
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"PublishGoal", {}}, "/world_model"),
               std::invalid_argument);
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"GoToPose", {{"x", "1"}}}, "/world_model"),
               std::invalid_argument);   // y has no default
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"GoToPose", {{"x", "1"}, {"y", "a"}}},
                              "/world_model"),
               std::invalid_argument);
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"PushBall", {{"pose", "1"}}}, "/world_model"),
               std::invalid_argument);   // world ports are wired, not set
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"Attack", {{"x", "1"}}}, "/world_model"),
               std::invalid_argument);
  EXPECT_THROW(buildVerifyXml(factory_, "V", NodeCall{"GoToPose", {{"x", "nan"}, {"y", "0"}}},
                              "/world_model"),
               std::invalid_argument);
}

TEST_F(TreeTest, VerifyRunsRealGoToPose)
{
  const auto xml = pinky_soccer_bt::buildVerifyXml(
      factory_, "VerifyGoTo", {"GoToPose", {{"x", "0.5"}, {"y", "-0.2"}, {"theta", "1.0"}}},
      "/world_model");
  factory_.registerBehaviorTreeFromText(xml);
  auto tree = factory_.createTree("VerifyGoTo", bb_);
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  auto g = decideWith(tree, world(0, 0, 0, 0.3, 0, TrackedObject::LOST, false), &status);
  EXPECT_EQ(g.action, "GoToPose");
  EXPECT_DOUBLE_EQ(g.target.x, 0.5);
  EXPECT_DOUBLE_EQ(g.target.y, -0.2);
  EXPECT_DOUBLE_EQ(g.target.theta, 1.0);
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);   // runs without /game/playing
  g = decideWith(tree, world(0.49, -0.2, 0, 0.3, 0, TrackedObject::LOST, false), &status);
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);   // in place but heading 0 != 1.0
  g = decideWith(tree, world(0.49, -0.2, 0.95, 0.3, 0, TrackedObject::LOST, false), &status);
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);  // pose alone cannot terminate the skill
  ASSERT_NE(g.goal_id, 0u);
  MotionResult result;
  result.goal_id = g.goal_id + 1;
  result.state = MotionResult::SUCCESS;
  bb_->set("motion_result", ReceivedMotionResult{result, std::chrono::steady_clock::now()});
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING); // reject a different invocation
  result.goal_id = g.goal_id;
  bb_->set("motion_result", ReceivedMotionResult{result, std::chrono::steady_clock::now()});
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(bb_->get("goal", g));
  EXPECT_EQ(g.mode, BehaviorGoal::STOP);
}

TEST_F(TreeTest, VerifySubtreeAndDetourPath)
{
  factory_.registerBehaviorTreeFromText(pinky_soccer_bt::buildVerifyXml(
      factory_, "VerifyAttack", {"Attack", {}}, "/world_model"));
  auto tree = factory_.createTree("VerifyAttack", bb_);
  // Robot in front of the ball: GoBehindBall detours, planned_path = waypoint then behind pose.
  const auto g = decideWith(tree, world(0.9, 0.0, 0, 0.5, 0));
  EXPECT_EQ(g.action, "GoBehindBall");
  ASSERT_EQ(g.planned_path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(g.planned_path.poses[0].pose.position.x, g.target.x);
  EXPECT_NEAR(g.planned_path.poses[1].pose.position.x, 0.35, 1e-9);   // 0.15 behind the ball
}

TEST_F(TreeTest, VerifyProbeBuilds)
{
  const auto xml = pinky_soccer_bt::buildProbeXml(
      factory_, "VerifyProbeT",
      {{"BallKnown", {{"max_sigma", "0.3"}}}, {"IsBehindBall", {{"tol_dist", "0.08"}}}},
      "/world_model");
  factory_.registerBehaviorTreeFromText(xml);
  EXPECT_NO_THROW(factory_.createTree("VerifyProbeT", BT::Blackboard::create()));
  EXPECT_THROW(pinky_soccer_bt::buildProbeXml(factory_, "X", {{"GoToPose", {}}}, "/world_model"),
               std::invalid_argument);   // actions are not conditions
}

// ---- referee: after a goal (/game/kickoff true, not playing) return to the kickoff pose --
TEST_F(TreeTest, KickoffReturnsToStartPose)
{
  auto kick = node_->create_publisher<std_msgs::msg::Bool>("/game/kickoff", 10);
  std_msgs::msg::Bool on;
  on.data = true;
  BehaviorGoal g;
  for(int i = 0; i < 50 && g.action != "GoToPose"; ++i)
  {
    kick->publish(on);
    g = decide(world(0.5, 0.3, 0, 0.6, 0, TrackedObject::OBSERVED, false));
  }
  EXPECT_EQ(g.action, "GoToPose");
  EXPECT_DOUBLE_EQ(g.target.x, -0.4);
  EXPECT_DOUBLE_EQ(g.target.y, 0.1);
  EXPECT_DOUBLE_EQ(g.target.theta, 0.5);
  std_msgs::msg::Bool off;
  for(int i = 0; i < 50 && g.action != "StopRobot"; ++i)
  {
    kick->publish(off);
    g = decide(world(0.5, 0.3, 0, 0.6, 0, TrackedObject::OBSERVED, false));
  }
  EXPECT_EQ(g.action, "StopRobot");   // paused without kickoff: stop in place
}

TEST_F(TreeTest, HandoffSuccessDoesNotInsertStopButExplicitHaltStillStops)
{
  factory_.registerBehaviorTreeFromText(buildVerifyXml(factory_, "HandoffGoTo",
      {"GoToPose", {{"x", "0.1"}, {"y", "0"}, {"completion_mode", "HANDOFF"},
       {"next_ready", "true"}, {"next_x", "0.3"}}}, "/world_model"));
  auto tree = factory_.createTree("HandoffGoTo", bb_);
  auto g = decideWith(tree, world(0, 0, 0, 0.3, 0, TrackedObject::LOST, false));
  MotionResult result; result.goal_id = g.goal_id; result.state = MotionResult::SUCCESS;
  result.reason = "handoff_ready";
  bb_->set("motion_result", ReceivedMotionResult{result, std::chrono::steady_clock::now()});
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(bb_->get("goal", g));
  EXPECT_EQ(g.mode, BehaviorGoal::NAVIGATE);
  // An explicitly canceled running request retains the original stop behavior.
  tree.haltTree();
  bb_->set("motion_result", ReceivedMotionResult{});
  tree.tickOnce();tree.haltTree();
  ASSERT_TRUE(bb_->get("goal", g));
  EXPECT_EQ(g.mode, BehaviorGoal::STOP);
}

TEST_F(TreeTest, AttackOnlyBoundsBlindPushAndExplicitlyBacksUp)
{
  const char* xml = R"(<root BTCPP_format="4" main_tree_to_execute="ProbeAttack">
    <BehaviorTree ID="ProbeAttack"><AttackBall ball="{attack_ball}" pose="{attack_pose}"
      opp_goal="{attack_goal}"/></BehaviorTree></root>)";
  auto probe = factory_.createTreeFromText(xml, bb_);
  auto w = world(-0.15, 0., 0., 0., 0.);
  bb_->set("attack_pose", w.self); bb_->set("attack_ball", w.ball);
  geometry_msgs::msg::Point goal; goal.x = .83;
  bb_->set("attack_goal", goal); bb_->set("world_stamp", 1.0);
  probe.tickOnce();
  EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action, "PushBall");
  w.self.pose.x = .03; w.ball.state = TrackedObject::ESTIMATED;
  bb_->set("attack_pose", w.self); bb_->set("attack_ball", w.ball);
  bb_->set("world_stamp", 1.2);
  probe.tickOnce();
  EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action, "PushBall");
  bb_->set("world_stamp", 2.5);
  probe.tickOnce();
  auto command = bb_->get<BehaviorGoal>("goal");
  EXPECT_EQ(command.action, "Backoff");
  EXPECT_NEAR(command.target.x, -.13, 1e-6);
  probe.haltTree();
  w.ball.state = TrackedObject::LOST;
  bb_->set("attack_ball", w.ball);
  probe.tickOnce();
  EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action, "SpinInPlace");
}

TEST_F(TreeTest, AttackKeepsPushingWithCurrentPartialArcEvidence)
{
  const char* xml = R"(<root BTCPP_format="4" main_tree_to_execute="ProbeAttack">
    <BehaviorTree ID="ProbeAttack"><AttackBall ball="{attack_ball}" pose="{attack_pose}"
      opp_goal="{attack_goal}"/></BehaviorTree></root>)";
  auto probe = factory_.createTreeFromText(xml, bb_);
  auto w = world(-0.15, 0., 0., 0., 0.);
  bb_->set("attack_pose", w.self); bb_->set("attack_ball", w.ball);
  geometry_msgs::msg::Point goal; goal.x = .83;
  bb_->set("attack_goal", goal); bb_->set("world_stamp", 1.0);
  probe.tickOnce();
  w.ball.state = TrackedObject::ESTIMATED;
  bb_->set("attack_ball", w.ball); bb_->set("ball_contact", true);
  bb_->set("world_stamp", 5.0);
  probe.tickOnce();
  EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action, "PushBall");
  bb_->set("ball_contact", false); bb_->set("world_stamp", 7.0);
  probe.tickOnce();
  EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action, "Backoff");
  bb_->set("world_stamp", 10.0);
  probe.tickOnce();
  EXPECT_NE(bb_->get<BehaviorGoal>("goal").action, "Backoff");
}

TEST_F(TreeTest, AttackApproachRetainsOrbitSideThroughSmallPoseNoise)
{
  const char* xml = R"(<root BTCPP_format="4" main_tree_to_execute="ProbeAttack">
    <BehaviorTree ID="ProbeAttack"><AttackBall ball="{attack_ball}" pose="{attack_pose}"
      opp_goal="{attack_goal}"/></BehaviorTree></root>)";
  auto probe = factory_.createTreeFromText(xml, bb_);
  auto w = world(.1, .003, 0., 0., 0.);
  bb_->set("attack_ball", w.ball); bb_->set("attack_pose", w.self);
  geometry_msgs::msg::Point goal;goal.x=.83;
  bb_->set("attack_goal",goal);bb_->set("world_stamp",1.);
  probe.tickOnce();
  EXPECT_GT(bb_->get<BehaviorGoal>("goal").target.y,0.);
  w.self.pose.y=-.003;bb_->set("attack_pose",w.self);bb_->set("world_stamp",1.1);
  probe.tickOnce();
  EXPECT_GT(bb_->get<BehaviorGoal>("goal").target.y,0.);
  w.self.pose.y=-.12;bb_->set("attack_pose",w.self);bb_->set("world_stamp",1.2);
  probe.tickOnce();
  EXPECT_LT(bb_->get<BehaviorGoal>("goal").target.y,0.);
}

TEST_F(TreeTest, AttackAlignsInPlaceWhenAlreadyBehindBall)
{
  const char* xml = R"(<root BTCPP_format="4" main_tree_to_execute="ProbeAttack">
    <BehaviorTree ID="ProbeAttack"><AttackBall ball="{attack_ball}" pose="{attack_pose}"
      opp_goal="{attack_goal}"/></BehaviorTree></root>)";
  auto probe = factory_.createTreeFromText(xml, bb_);
  auto w = world(-.1, 0., .4, 0., 0.);
  bb_->set("attack_ball", w.ball); bb_->set("attack_pose", w.self);
  geometry_msgs::msg::Point goal;goal.x=.83;
  bb_->set("attack_goal",goal);bb_->set("world_stamp",1.);
  probe.tickOnce();
  const auto g=bb_->get<BehaviorGoal>("goal");
  EXPECT_EQ(g.action,"AlignToBall");EXPECT_NEAR(g.target.x,-.1,1e-6);
  EXPECT_NEAR(g.target.theta,0.,1e-6);
  w.self.pose.theta=.1;bb_->set("attack_pose",w.self);bb_->set("world_stamp",1.1);
  probe.tickOnce();EXPECT_EQ(bb_->get<BehaviorGoal>("goal").action,"PushBall");
}
