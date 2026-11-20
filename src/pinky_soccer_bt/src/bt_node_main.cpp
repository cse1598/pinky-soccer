// bt_node: loads trees/soccer_1v1.xml and ticks it at a fixed rate.
// Input /world_model (via ReadWorldModel), output /behavior_goal (via PublishBehaviorGoal).
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pinky_soccer_bt/bt_nodes.hpp"

using pinky_soccer_bt::BehaviorGoal;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_node");
  const auto share = ament_index_cpp::get_package_share_directory("pinky_soccer_bt");
  const auto tree_file = node->declare_parameter<std::string>(
      "tree_file", share + "/trees/soccer_1v1.xml");
  const double rate = node->declare_parameter<double>("tick_rate_hz", 15.0);
  const bool groot2 = node->declare_parameter<bool>("groot2_enabled", true);
  const int groot2_port = node->declare_parameter<int>("groot2_port", 1667);
  const bool cout_logger = node->declare_parameter<bool>("cout_logger", false);
  const auto file_log = node->declare_parameter<std::string>("file_logger_path", "");
  const auto wm_topic = node->declare_parameter<std::string>("world_model_topic", "/world_model");
  const auto goal_topic =
      node->declare_parameter<std::string>("behavior_goal_topic", "/behavior_goal");
  if(rate <= 0.0)
  {
    RCLCPP_FATAL(node->get_logger(), "tick_rate_hz must be positive");
    return 1;
  }

  BT::BehaviorTreeFactory factory;
  pinky_soccer_bt::registerSoccerNodes(factory, BT::RosNodeParams(node, wm_topic),
                                       BT::RosNodeParams(node, goal_topic));
  factory.registerBehaviorTreeFromFile(tree_file);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("goal", pinky_soccer_bt::makeGoal("STOP", "Init", BehaviorGoal::STOP,
                                                    pinky_soccer_bt::Pose2{}, 0.0));
  auto tree = factory.createTree("Main", blackboard);
  auto publish_tree = factory.createTree("PublishGoal", blackboard);

  std::unique_ptr<BT::Groot2Publisher> groot;
  std::unique_ptr<BT::StdCoutLogger> cout_log;
  std::unique_ptr<BT::FileLogger2> file_logger;
  if(groot2)
  {
    groot = std::make_unique<BT::Groot2Publisher>(tree, groot2_port);
  }
  if(cout_logger)
  {
    cout_log = std::make_unique<BT::StdCoutLogger>(tree);
  }
  if(!file_log.empty())
  {
    file_logger = std::make_unique<BT::FileLogger2>(tree, file_log);
  }
  RCLCPP_INFO(node->get_logger(), "Loaded %s, ticking at %.1f Hz (Groot2 %s)", tree_file.c_str(),
              rate, groot2 ? std::to_string(groot2_port).c_str() : "off");

  std::string last_action;
  auto timer = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [&]() {
        tree.tickOnce();
        publish_tree.tickOnce();
        BehaviorGoal goal;
        if(blackboard->get("goal", goal) && goal.branch + "/" + goal.action != last_action)
        {
          last_action = goal.branch + "/" + goal.action;
          RCLCPP_INFO(node->get_logger(), "goal: %s", last_action.c_str());
        }
      });
  rclcpp::spin(node);
  tree.haltTree();
  rclcpp::shutdown();
  return 0;
}
