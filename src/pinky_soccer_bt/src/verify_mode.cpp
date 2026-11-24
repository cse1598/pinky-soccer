#include "pinky_soccer_bt/verify_mode.hpp"

#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/xml_parsing.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

#include "pinky_soccer_bt/bt_nodes.hpp"

namespace pinky_soccer_bt
{

using json = nlohmann::json;

namespace
{
const std::set<std::string> kInfrastructure{"ReadWorldModel", "PublishBehaviorGoal"};
const std::set<std::string> kHiddenTrees{"PublishGoal"};

std::string escape(const std::string& s)
{
  std::string out;
  for(char c : s)
  {
    switch(c)
    {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

bool isWorldPort(const std::string& name)
{
  const auto& w = worldPorts();
  return std::find(w.begin(), w.end(), name) != w.end();
}

std::string numberLiteral(const std::string& port, const std::string& value)
{
  std::size_t used = 0;
  double v = 0.0;
  try
  {
    v = std::stod(value, &used);
  }
  catch(const std::exception&)
  {
    used = 0;
  }
  if(used == 0 || used != value.size() || !std::isfinite(v))
  {
    throw std::invalid_argument("port '" + port + "' needs a finite number, got '" + value + "'");
  }
  std::ostringstream os;
  os.precision(17);
  os << v;
  return os.str();
}

std::string readWorldModelXml(const std::string& topic)
{
  std::string xml = "<ReadWorldModel topic_name=\"" + escape(topic) + "\"";
  for(const auto& key : worldPorts())
  {
    xml += " " + key + "=\"{" + key + "}\"";
  }
  return xml + "/>";
}

// Element for an action/condition node: literal values for requested ports, blackboard
// references for world ports, defaults (from the C++ manifest) for everything else.
std::string nodeXml(const BT::BehaviorTreeFactory& factory, const NodeCall& call,
                    BT::NodeType expected, const std::string& extra_attrs)
{
  const auto it = factory.manifests().find(call.target);
  if(it == factory.manifests().end() || it->second.type != expected ||
     kInfrastructure.count(call.target))
  {
    throw std::invalid_argument("unknown or not runnable node '" + call.target + "'");
  }
  const auto& ports = it->second.ports;
  std::string xml = "<" + call.target + extra_attrs;
  std::set<std::string> given;
  for(const auto& [name, value] : call.ports)
  {
    if(!ports.count(name) || isWorldPort(name))
    {
      throw std::invalid_argument("node '" + call.target + "' has no settable port '" + name +
                                  "'");
    }
    given.insert(name);
    xml += " " + name + "=\"" + numberLiteral(name, value) + "\"";
  }
  for(const auto& [name, info] : ports)
  {
    if(isWorldPort(name))
    {
      xml += " " + name + "=\"{" + name + "}\"";
    }
    else if(!given.count(name) && info.defaultValueString().empty())
    {
      throw std::invalid_argument("node '" + call.target + "' needs a value for port '" + name +
                                  "'");
    }
  }
  return xml + "/>";
}

std::string statusName(BT::NodeStatus s)
{
  return BT::toStr(s, false);
}

void collectNodes(const BT::TreeNode* node, int depth, const BT::TreeObserver* observer,
                  json& out)
{
  if(node == nullptr)
  {
    return;
  }
  json entry{{"depth", depth},
             {"name", node->name()},
             {"type", node->registrationName()},
             {"status", statusName(node->status())}};
  if(observer != nullptr)
  {
    const auto& stats = observer->statistics();
    const auto found = stats.find(node->UID());
    if(found != stats.end())
    {
      entry["last"] = statusName(found->second.last_result);
      entry["success"] = found->second.success_count;
      entry["failure"] = found->second.failure_count;
    }
  }
  out.push_back(entry);
  if(const auto* control = dynamic_cast<const BT::ControlNode*>(node))
  {
    for(const auto* child : control->children())
    {
      collectNodes(child, depth + 1, observer, out);
    }
  }
  else if(const auto* decorator = dynamic_cast<const BT::DecoratorNode*>(node))
  {
    collectNodes(decorator->child(), depth + 1, observer, out);
  }
}

double seconds(std::chrono::steady_clock::duration d)
{
  return std::chrono::duration<double>(d).count();
}

std::vector<std::pair<std::string, std::string>> portsFrom(const json& j)
{
  std::vector<std::pair<std::string, std::string>> out;
  if(!j.is_object())
  {
    return out;
  }
  for(const auto& [key, value] : j.items())
  {
    out.emplace_back(key, value.is_string() ? value.get<std::string>() : value.dump());
  }
  return out;
}
}  // namespace

const std::vector<std::string>& worldPorts()
{
  static const std::vector<std::string> ports{"playing", "pose",     "ball",
                                              "opponents", "own_goal", "opp_goal"};
  return ports;
}

std::vector<std::string> runnableTargets(const BT::BehaviorTreeFactory& factory)
{
  std::vector<std::string> out;
  for(const auto& [id, manifest] : factory.manifests())
  {
    if(manifest.type == BT::NodeType::ACTION && !kInfrastructure.count(id) &&
       factory.builtinNodes().count(id) == 0)
    {
      out.push_back(id);
    }
  }
  for(const auto& id : factory.registeredBehaviorTrees())
  {
    if(!kHiddenTrees.count(id) && id.rfind("Verify", 0) != 0)
    {
      out.push_back(id);
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::string buildVerifyXml(const BT::BehaviorTreeFactory& factory, const std::string& tree_id,
                           const NodeCall& call, const std::string& world_model_topic)
{
  const auto trees = factory.registeredBehaviorTrees();
  std::string body;
  if(std::find(trees.begin(), trees.end(), call.target) != trees.end() &&
     !kHiddenTrees.count(call.target) && call.target.rfind("Verify", 0) != 0)
  {
    if(!call.ports.empty())
    {
      throw std::invalid_argument("subtree '" + call.target + "' takes no ports");
    }
    body = "<SubTree ID=\"" + call.target + "\" _autoremap=\"true\"/>";
  }
  else
  {
    body = nodeXml(factory, call, BT::NodeType::ACTION, "");
  }
  return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"" + tree_id + "\"><ReactiveSequence>" +
         readWorldModelXml(world_model_topic) + body + "</ReactiveSequence></BehaviorTree></root>";
}

std::string buildProbeXml(const BT::BehaviorTreeFactory& factory, const std::string& tree_id,
                          const std::vector<NodeCall>& conditions,
                          const std::string& world_model_topic)
{
  std::string xml = "<root BTCPP_format=\"4\"><BehaviorTree ID=\"" + tree_id + "\"><Sequence>" +
                    readWorldModelXml(world_model_topic);
  for(std::size_t i = 0; i < conditions.size(); ++i)
  {
    xml += "<ForceSuccess>" +
           nodeXml(factory, conditions[i], BT::NodeType::CONDITION,
                   " name=\"probe_" + std::to_string(i) + "\"") +
           "</ForceSuccess>";
  }
  return xml + "</Sequence></BehaviorTree></root>";
}

// ---------------------------------------------------------------- VerifyRunner
VerifyRunner::VerifyRunner(rclcpp::Node::SharedPtr node, BT::BehaviorTreeFactory& factory,
                           BT::Blackboard::Ptr blackboard, std::string world_model_topic,
                           double heartbeat_timeout)
  : node_(std::move(node)), factory_(factory), blackboard_(std::move(blackboard)),
    wm_topic_(std::move(world_model_topic)), heartbeat_timeout_(heartbeat_timeout)
{
  const auto reliable = rclcpp::QoS(10).reliable();
  status_pub_ = node_->create_publisher<std_msgs::msg::String>("/behavior_test/status", reliable);
  models_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/behavior_test/models", rclcpp::QoS(1).reliable().transient_local());
  request_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/behavior_test/request", reliable,
      [this](const std_msgs::msg::String& msg) { onRequest(msg); });
  std_msgs::msg::String models;
  json catalog{{"models", BT::writeTreeNodesModelXML(factory_, false)},
               {"runnable", runnableTargets(factory_)}};
  models.data = catalog.dump();
  models_pub_->publish(models);
  setStop("VerifyIdle");
  RCLCPP_WARN(node_->get_logger(), "VERIFY MODE: the game tree is not ticked; behaviours run "
                                   "only on /behavior_test/request");
}

void VerifyRunner::setStop(const std::string& action)
{
  blackboard_->set("goal", makeGoal("STOP", action, BehaviorGoal::STOP, Pose2{}, 0.0));
}

void VerifyRunner::onRequest(const std_msgs::msg::String& msg)
{
  last_request_ = std::chrono::steady_clock::now();
  json req;
  try
  {
    req = json::parse(msg.data);
  }
  catch(const std::exception& e)
  {
    RCLCPP_WARN(node_->get_logger(), "verify: bad request JSON: %s", e.what());
    return;
  }
  const auto command = req.value("command", std::string{});
  if(command == "ping")
  {
    return;
  }
  if(command == "stop")
  {
    if(tree_)
    {
      finish("stopped", "stopped by the verifier");
    }
    setStop("VerifyStop");
    return;
  }
  if(command == "probe")
  {
    std::vector<NodeCall> calls;
    probe_names_.clear();
    for(const auto& c : req.value("conditions", json::array()))
    {
      calls.push_back({c.value("target", std::string{}), portsFrom(c.value("ports", json{}))});
      probe_names_.push_back(c.value("label", calls.back().target));
    }
    try
    {
      const std::string id = "VerifyProbe" + std::to_string(++tree_counter_);
      factory_.registerBehaviorTreeFromText(buildProbeXml(factory_, id, calls, wm_topic_));
      probe_.emplace(factory_.createTree(id, BT::Blackboard::create()));
      probe_observer_ = std::make_unique<BT::TreeObserver>(*probe_);
    }
    catch(const std::exception& e)
    {
      probe_.reset();
      probe_observer_.reset();
      probe_names_.clear();
      RCLCPP_WARN(node_->get_logger(), "verify: probe rejected: %s", e.what());
    }
    return;
  }
  if(command == "run")
  {
    const NodeCall call{req.value("target", std::string{}), portsFrom(req.value("ports", json{}))};
    start(req.value("id", -1), call, req.value("timeout_sec", 30.0));
    return;
  }
  RCLCPP_WARN(node_->get_logger(), "verify: unknown command '%s'", command.c_str());
}

void VerifyRunner::start(int id, const NodeCall& call, double timeout_sec)
{
  if(tree_)
  {
    finish("stopped", "replaced by a new request");
  }
  request_id_ = id;
  target_ = call.target;
  ticks_ = 0;
  elapsed_ = 0.0;
  timeout_sec_ = std::clamp(timeout_sec, 1.0, 600.0);
  nodes_json_ = "[]";
  try
  {
    const std::string tree_id = "Verify" + std::to_string(++tree_counter_);
    factory_.registerBehaviorTreeFromText(buildVerifyXml(factory_, tree_id, call, wm_topic_));
    tree_.emplace(factory_.createTree(tree_id, blackboard_));
    observer_ = std::make_unique<BT::TreeObserver>(*tree_);
  }
  catch(const std::exception& e)
  {
    tree_.reset();
    observer_.reset();
    state_ = "error";
    message_ = e.what();
    setStop("VerifyError");
    RCLCPP_WARN(node_->get_logger(), "verify: request %d rejected: %s", id, e.what());
    return;
  }
  started_ = node_->now();
  state_ = "running";
  message_.clear();
  RCLCPP_INFO(node_->get_logger(), "verify: run %d %s (timeout %.0f s)", id, target_.c_str(),
              timeout_sec_);
}

void VerifyRunner::finish(const std::string& state, const std::string& message)
{
  if(tree_)
  {
    json nodes = json::array();
    collectNodes(tree_->rootNode(), 0, observer_.get(), nodes);
    nodes_json_ = nodes.dump();
    tree_->haltTree();
  }
  observer_.reset();
  tree_.reset();
  state_ = state;
  message_ = message;
  setStop("VerifyDone");
  RCLCPP_INFO(node_->get_logger(), "verify: run %d %s -> %s %s", request_id_, target_.c_str(),
              state.c_str(), message.c_str());
}

void VerifyRunner::tick()
{
  const auto now = std::chrono::steady_clock::now();
  probe_ok_ = false;
  if(probe_)
  {
    probe_ok_ = probe_->tickOnce() == BT::NodeStatus::SUCCESS;
  }
  if(tree_)
  {
    elapsed_ = (node_->now() - started_).seconds();
    if(seconds(now - last_request_) > heartbeat_timeout_)
    {
      finish("verifier_lost", "no request/ping from the verifier");
    }
    else if(elapsed_ > timeout_sec_)
    {
      finish("timeout", "time limit reached");
    }
    else
    {
      const auto status = tree_->tickOnce();
      ++ticks_;
      BehaviorGoal goal;
      const bool stale = blackboard_->get("goal", goal) &&
                         (goal.action == "NoWorldModel" || goal.action == "WorldModelStale");
      json nodes = json::array();
      collectNodes(tree_->rootNode(), 0, observer_.get(), nodes);
      nodes_json_ = nodes.dump();
      if(stale)
      {
        finish("world_model_stale", goal.action);
      }
      else if(status == BT::NodeStatus::SUCCESS)
      {
        finish("success", "");
      }
      else if(status == BT::NodeStatus::FAILURE)
      {
        finish("failure", "");
      }
    }
  }
  publishStatus();
}

void VerifyRunner::publishStatus()
{
  json status{{"verify", true},
              {"request_id", request_id_},
              {"target", target_},
              {"state", state_},
              {"message", message_},
              {"elapsed", elapsed_},
              {"ticks", ticks_},
              {"timeout_sec", timeout_sec_},
              {"nodes", json::parse(nodes_json_)}};
  BehaviorGoal goal;
  if(blackboard_->get("goal", goal))
  {
    status["goal"] = {{"branch", goal.branch}, {"action", goal.action},
                      {"mode", goal.mode},     {"x", goal.target.x},
                      {"y", goal.target.y},    {"theta", goal.target.theta},
                      {"max_speed", goal.max_speed}};
  }
  json conditions = json::array();
  if(probe_ && probe_observer_)
  {
    std::vector<std::optional<bool>> values(probe_names_.size());
    probe_->applyVisitor([&](BT::TreeNode* n) {
      if(n->name().rfind("probe_", 0) == 0)
      {
        const auto index = std::stoul(n->name().substr(6));
        const auto& stats = probe_observer_->statistics();
        const auto found = stats.find(n->UID());
        if(probe_ok_ && index < values.size() && found != stats.end() &&
           found->second.last_result != BT::NodeStatus::IDLE)
        {
          values[index] = found->second.last_result == BT::NodeStatus::SUCCESS;
        }
      }
    });
    for(std::size_t i = 0; i < probe_names_.size(); ++i)
    {
      conditions.push_back({{"name", probe_names_[i]},
                            {"value", values[i] ? json(*values[i]) : json(nullptr)}});
    }
  }
  status["conditions"] = conditions;
  std_msgs::msg::String msg;
  msg.data = status.dump();
  status_pub_->publish(msg);
}

}  // namespace pinky_soccer_bt
