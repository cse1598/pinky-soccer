// Behaviour tree nodes for Pinky soccer. The tree reads only /world_model and writes a
// BehaviorGoal into the root blackboard entry "goal"; PublishBehaviorGoal sends it.
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <pinky_soccer_msgs/msg/behavior_goal.hpp>
#include <pinky_soccer_msgs/msg/tracked_object.hpp>
#include <pinky_soccer_msgs/msg/world_model.hpp>

#include "pinky_soccer_bt/soccer_logic.hpp"

namespace pinky_soccer_bt
{

using pinky_soccer_msgs::msg::BehaviorGoal;
using pinky_soccer_msgs::msg::TrackedObject;
using pinky_soccer_msgs::msg::WorldModel;
using geometry_msgs::msg::Point;
using Opponents = std::vector<TrackedObject>;

// Root blackboard key holding the current BehaviorGoal.
inline constexpr const char* kGoalKey = "@goal";

BehaviorGoal makeGoal(const std::string& branch, const std::string& action, uint8_t mode,
                      const Pose2& target, double max_speed);
Pose2 toPose(const TrackedObject& o);
Vec2 toVec(const TrackedObject& o);
Vec2 toVec(const Point& p);

// Copies the latest /world_model into blackboard ports. FAILURE (and a STOP goal) when no
// message arrived yet or the latest one is older than `stale_sec` (steady clock).
class ReadWorldModel : public BT::RosTopicSubNode<WorldModel>
{
public:
  ReadWorldModel(const std::string& name, const BT::NodeConfig& conf,
                 const BT::RosNodeParams& params);
  static BT::PortsList providedPorts();
  BT::NodeStatus onTick(const std::shared_ptr<WorldModel>& last_msg) override;
  bool latchLastMessage() const override { return true; }

private:
  std::shared_ptr<WorldModel> seen_;
  std::chrono::steady_clock::time_point received_{};
};

// Publishes the blackboard goal (ticked as its own tree after the main tree).
class PublishBehaviorGoal : public BT::RosTopicPubNode<BehaviorGoal>
{
public:
  PublishBehaviorGoal(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params);
  static BT::PortsList providedPorts();
  bool setMessage(BehaviorGoal& msg) override;
};

// ---- conditions ------------------------------------------------------------------
class IsPlaying : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class BallKnown : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class BallInOwnHalf : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class OpponentCloserToBall : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class IsBehindBall : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class NotFacingOwnGoal : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ---- actions: each tick computes a goal and stores it under kGoalKey ---------------
class GoalAction : public BT::StatefulActionNode
{
public:
  GoalAction(const std::string& name, const BT::NodeConfig& conf, std::string branch);
  BT::NodeStatus onStart() override { return update(); }
  BT::NodeStatus onRunning() override { return update(); }
  void onHalted() override {}

protected:
  virtual BT::NodeStatus update() = 0;
  void setGoal(uint8_t mode, const Pose2& target, double max_speed);
  template <typename T>
  T input(const std::string& port)
  {
    auto value = getInput<T>(port);
    if(!value)
    {
      throw BT::RuntimeError(registrationName(), ": missing input [", port,
                             "]: ", value.error());
    }
    return value.value();
  }
  std::string branch_;
};

class StopRobot : public GoalAction
{
public:
  StopRobot(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "STOP") {}
  static BT::PortsList providedPorts() { return {}; }

protected:
  BT::NodeStatus update() override;
};

class GoBehindBall : public GoalAction
{
public:
  GoBehindBall(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "ATTACK") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

class PushBall : public GoalAction
{
public:
  PushBall(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "ATTACK") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

class GoToBlockPoint : public GoalAction
{
public:
  GoToBlockPoint(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "DEFEND") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;

private:
  bool in_place_{false};   // hysteresis: reached at tol, left at 2 * tol
};

class FaceBall : public GoalAction
{
public:
  FaceBall(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "DEFEND") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

class TurnToLastSeen : public GoalAction
{
public:
  TurnToLastSeen(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "SEARCH") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

class SpinInPlace : public GoalAction
{
public:
  SpinInPlace(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "SEARCH") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

class GoToPose : public GoalAction
{
public:
  GoToPose(const std::string& n, const BT::NodeConfig& c) : GoalAction(n, c, "SEARCH") {}
  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus update() override;
};

// Registers every node used by trees/soccer_1v1.xml.
void registerSoccerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& world_model,
                         const BT::RosNodeParams& goal);

}  // namespace pinky_soccer_bt
