// bt_node: loads trees/soccer_1v1.xml and ticks it at a fixed rate.
// Input /world_model (via ReadWorldModel), output /behavior_goal (via PublishBehaviorGoal).
// verify_mode:=true ticks single behaviours requested by the verifier UI instead of "Main".
// `bt_node --print-models` prints the node models (ports, defaults) and runnable targets.
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/xml_parsing.h>

#include <cstring>
#include <iostream>

#include "pinky_soccer_bt/bt_nodes.hpp"
#include "pinky_soccer_bt/verify_mode.hpp"

using pinky_soccer_bt::BehaviorGoal;

int main(int argc, char** argv)
{
  if(argc > 1 && std::strcmp(argv[1], "--print-models") == 0)
  {
    rclcpp::init(1, argv);
    auto probe = std::make_shared<rclcpp::Node>("bt_node_models");
    BT::BehaviorTreeFactory factory;
    pinky_soccer_bt::registerSoccerNodes(factory, BT::RosNodeParams(probe, "/world_model"),
                                         BT::RosNodeParams(probe, "/behavior_goal"));
    factory.registerBehaviorTreeFromFile(
        argc > 2 ? argv[2] :
                   ament_index_cpp::get_package_share_directory("pinky_soccer_bt") +
                       "/trees/soccer_1v1.xml");
    std::cout << BT::writeTreeNodesModelXML(factory, false) << "\n<!-- runnable:";
    for(const auto& id : pinky_soccer_bt::runnableTargets(factory))
    {
      std::cout << " " << id;
    }
    std::cout << " -->" << std::endl;
    rclcpp::shutdown();
    return 0;
  }
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
  const auto wm_topic = node->declare_parameter<std::string>("world_model_topic", "world_model");
  const auto goal_topic =
      node->declare_parameter<std::string>("behavior_goal_topic", "behavior_goal");
  const bool verify_mode = node->declare_parameter<bool>("verify_mode", false);
  const double heartbeat = node->declare_parameter<double>("verify_heartbeat_timeout", 1.5);
  // Composed behaviours (subtrees) stay off while single skills are verified one by one.
  const bool verify_subtrees = node->declare_parameter<bool>("verify_subtrees", false);
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
  auto motion_sub = node->create_subscription<pinky_soccer_bt::MotionResult>(
      "motion/result", 10,
      [blackboard](pinky_soccer_bt::MotionResult::SharedPtr msg) {
        blackboard->set("motion_result", pinky_soccer_bt::ReceivedMotionResult{
            *msg, std::chrono::steady_clock::now()});
      });
  // Kickoff pose (map frame): where the robot returns after a goal (/game/kickoff).
  const auto kickoff = node->declare_parameter<std::vector<double>>("kickoff_pose",
                                                                    {0.0, 0.0, 0.0});
  if(kickoff.size() != 3)
  {
    RCLCPP_FATAL(node->get_logger(), "kickoff_pose needs [x, y, theta]");
    return 1;
  }
  pinky_soccer_bt::setKickoffPose(blackboard, kickoff[0], kickoff[1], kickoff[2]);
  blackboard->set("goal", pinky_soccer_bt::makeGoal("STOP", "Init", BehaviorGoal::STOP,
                                                    pinky_soccer_bt::Pose2{}, 0.0));
  auto preparation_pose = node->create_subscription<geometry_msgs::msg::Pose2D>(
      "preparation/start_pose", rclcpp::QoS(1).transient_local(),
      [blackboard](geometry_msgs::msg::Pose2D::SharedPtr msg) {
        pinky_soccer_bt::setKickoffPose(blackboard, msg->x, msg->y, msg->theta);
      });
  auto tree = factory.createTree("Main", blackboard);
  auto publish_tree = factory.createTree("PublishGoal", blackboard);

  std::unique_ptr<pinky_soccer_bt::VerifyRunner> verifier;
  if(verify_mode)
  {
    verifier = std::make_unique<pinky_soccer_bt::VerifyRunner>(node, factory, blackboard,
                                                               wm_topic, heartbeat,
                                                               verify_subtrees);
  }

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
        if(verifier)
        {
          verifier->tick();
        }
        else
        {
          tree.tickOnce();
        }
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
