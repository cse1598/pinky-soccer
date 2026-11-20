// Loads the real tree XML, feeds /world_model messages and checks the chosen goal.
#include <gtest/gtest.h>

#include <cstdlib>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "pinky_soccer_bt/bt_nodes.hpp"

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
  BehaviorGoal decide(const WorldModel& w)
  {
    BehaviorGoal goal;
    for(int i = 0; i < 100; ++i)
    {
      pub_->publish(w);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      tree_.tickOnce();
      TrackedObject pose, ball;
      bool playing = false;
      if(bb_->get("pose", pose) && bb_->get("ball", ball) && bb_->get("playing", playing) &&
         pose.pose.x == w.self.pose.x && ball.pose.x == w.ball.pose.x &&
         playing == static_cast<bool>(w.playing))
      {
        tree_.tickOnce();
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
