// Verification mode for bt_node: instead of the game tree "Main", run exactly one requested
// action node or subtree from the same factory (the real C++ nodes and trees/soccer_1v1.xml),
// so a person can command and evaluate single behaviours from the behaviour verifier UI.
//
// Protocol (std_msgs/String with JSON, no new message types; topics relative to the robot
// namespace):
//   request  /behavior_test/request   {"command": "run", "id": 3, "target": "GoToPose",
//                                      "ports": {"x": "0.5", "y": "0.2"}, "timeout_sec": 30}
//                                     {"command": "stop"} | {"command": "ping"}
//                                     {"command": "probe", "conditions": [{"target": ..., "ports": {...}}]}
//   status   /behavior_test/status    JSON, every tick (state, active nodes, goal, conditions)
//   models   /behavior_test/models    TreeNodesModel XML (transient local): ports and defaults
// Safety: a running test stops (STOP goal) on timeout, on a stale world model, and when no
// request (any command, the UI pings) arrived for `heartbeat_timeout` seconds.
// Elapsed time and the time limit use the node clock (simulation time in simulation, so a
// slow simulator does not cut runs short); the heartbeat uses the steady clock.
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace pinky_soccer_bt
{

// One node or subtree with literal port values (numbers only; world ports are wired).
struct NodeCall
{
  std::string target;
  std::vector<std::pair<std::string, std::string>> ports;
};

// Blackboard keys written by ReadWorldModel. Ports with these names are wired automatically.
const std::vector<std::string>& worldPorts();

// Runnable targets: action nodes (except infrastructure) and subtrees (except PublishGoal).
std::vector<std::string> runnableTargets(const BT::BehaviorTreeFactory& factory);

// XML for one verification tree: ReactiveSequence(ReadWorldModel, target). Throws
// std::invalid_argument for an unknown target or port, or a non-numeric/non-finite value.
std::string buildVerifyXml(const BT::BehaviorTreeFactory& factory, const std::string& tree_id,
                           const NodeCall& call, const std::string& world_model_topic);

// XML for the condition probe: Sequence(ReadWorldModel, ForceSuccess(condition)...), each
// condition named "probe_<index>".
std::string buildProbeXml(const BT::BehaviorTreeFactory& factory, const std::string& tree_id,
                          const std::vector<NodeCall>& conditions,
                          const std::string& world_model_topic);

class VerifyRunner
{
public:
  VerifyRunner(rclcpp::Node::SharedPtr node, BT::BehaviorTreeFactory& factory,
               BT::Blackboard::Ptr blackboard, std::string world_model_topic,
               double heartbeat_timeout, bool allow_subtrees = false);
  // Called by bt_node every cycle instead of ticking "Main".
  void tick();

private:
  void onRequest(const std_msgs::msg::String& msg);
  void start(int id, const NodeCall& call, double timeout_sec);
  void finish(const std::string& state, const std::string& message);
  void setStop(const std::string& action);
  void publishStatus();

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory& factory_;
  BT::Blackboard::Ptr blackboard_;
  std::string wm_topic_;
  double heartbeat_timeout_;
  bool allow_subtrees_;   // composed behaviours (subtrees) runnable from the verifier
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr models_pub_;

  std::optional<BT::Tree> tree_;
  std::unique_ptr<BT::TreeObserver> observer_;
  std::optional<BT::Tree> probe_;
  std::unique_ptr<BT::TreeObserver> probe_observer_;
  std::vector<std::string> probe_names_;
  bool probe_ok_{false};

  int request_id_{-1};
  int tree_counter_{0};
  std::string target_;
  std::string state_{"idle"};
  std::string message_;
  unsigned ticks_{0};
  double timeout_sec_{0.0};
  rclcpp::Time started_;   // node clock (simulation time in simulation)
  std::chrono::steady_clock::time_point last_request_{};
  double elapsed_{0.0};
  std::string nodes_json_{"[]"};
};

}  // namespace pinky_soccer_bt
