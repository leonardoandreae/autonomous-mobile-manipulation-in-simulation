#include "amm_task_planner/task_planner_node.hpp"

#include <curl/curl.h>

#include <sstream>
#include <stdexcept>
#include <thread>

// Minimal JSON helpers — avoids an external nlohmann/json dependency.
// For a production build, replace with nlohmann/json or similar.
namespace json_util
{

/**
 * Extract the first string value for a given key from a flat JSON object.
 * e.g.  extract_string(R"({"object_name":"apple"})", "object_name") -> "apple"
 */
static std::string extract_string(const std::string & json, const std::string & key)
{
  const std::string search = "\"" + key + "\"";
  auto pos = json.find(search);
  if (pos == std::string::npos) {
    return "";
  }
  pos = json.find(':', pos);
  if (pos == std::string::npos) {
    return "";
  }
  pos = json.find('"', pos);
  if (pos == std::string::npos) {
    return "";
  }
  ++pos;
  auto end = json.find('"', pos);
  if (end == std::string::npos) {
    return "";
  }
  return json.substr(pos, end - pos);
}

/**
 * Extract the first integer value for a given key from a flat JSON object.
 */
static int extract_int(const std::string & json, const std::string & key, int default_val = 0)
{
  const std::string search = "\"" + key + "\"";
  auto pos = json.find(search);
  if (pos == std::string::npos) {
    return default_val;
  }
  pos = json.find(':', pos);
  if (pos == std::string::npos) {
    return default_val;
  }
  // skip whitespace
  while (++pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) {}
  if (pos >= json.size()) {
    return default_val;
  }
  return std::stoi(json.substr(pos));
}

/**
 * Extract the JSON object found inside the Ollama response field "response".
 * Ollama wraps the model output in: {"response":"<model output>", ...}
 * The model output itself is the JSON we care about.
 */
static std::string extract_response_field(const std::string & ollama_json)
{
  // Find the "response" key
  auto pos = ollama_json.find("\"response\"");
  if (pos == std::string::npos) {
    return ollama_json;  // fallback: treat whole thing as payload
  }
  pos = ollama_json.find(':', pos);
  if (pos == std::string::npos) {
    return "";
  }
  pos = ollama_json.find('"', pos);
  if (pos == std::string::npos) {
    return "";
  }
  ++pos;
  // Walk forward handling escaped quotes
  std::string result;
  while (pos < ollama_json.size()) {
    if (ollama_json[pos] == '\\' && pos + 1 < ollama_json.size()) {
      if (ollama_json[pos + 1] == '"') {
        result += '"';
        pos += 2;
        continue;
      } else if (ollama_json[pos + 1] == 'n') {
        result += '\n';
        pos += 2;
        continue;
      }
    }
    if (ollama_json[pos] == '"') {
      break;
    }
    result += ollama_json[pos++];
  }
  return result;
}

}  // namespace json_util

namespace amm_task_planner
{

// ── Constructor / Destructor ───────────────────────────────────────────────────

TaskPlannerNode::TaskPlannerNode(const rclcpp::NodeOptions & options)
: Node("task_planner_node", options)
{
  // Declare parameters (override in config/task_planner.yaml)
  declare_parameter("ollama_host",       "http://localhost:11434");
  declare_parameter("ollama_model",      "llama3.2");
  declare_parameter("request_timeout_s", 30);

  get_parameter("ollama_host",       ollama_host_);
  get_parameter("ollama_model",      ollama_model_);
  get_parameter("request_timeout_s", request_timeout_s_);

  // Publisher — lets other nodes react to parsed commands
  task_cmd_pub_ = create_publisher<amm_msgs::msg::TaskCommand>(
    "task_command", rclcpp::QoS(10));

  // Action server — entry point for natural-language task requests
  action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this,
    "execute_task",
    std::bind(&TaskPlannerNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskPlannerNode::handle_cancel,   this, std::placeholders::_1),
    std::bind(&TaskPlannerNode::handle_accepted, this, std::placeholders::_1));

  curl_global_init(CURL_GLOBAL_DEFAULT);

  RCLCPP_INFO(get_logger(),
    "TaskPlannerNode ready. LLM: %s @ %s",
    ollama_model_.c_str(), ollama_host_.c_str());
}

TaskPlannerNode::~TaskPlannerNode()
{
  curl_global_cleanup();
}

// ── Action server callbacks ────────────────────────────────────────────────────

rclcpp_action::GoalResponse TaskPlannerNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteTask::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received prompt: \"%s\"", goal->prompt.c_str());
  if (goal->prompt.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting empty prompt.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskPlannerNode::handle_cancel(
  std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancel requested.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskPlannerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

void TaskPlannerNode::execute(std::shared_ptr<GoalHandle> goal_handle)
{
  auto result   = std::make_shared<ExecuteTask::Result>();
  auto feedback = std::make_shared<ExecuteTask::Feedback>();

  const auto & prompt = goal_handle->get_goal()->prompt;

  // ── Step 1: query the local LLM ─────────────────────────────────────────────
  feedback->status = "Querying LLM...";
  goal_handle->publish_feedback(feedback);

  std::string llm_json;
  try {
    llm_json = query_ollama(prompt);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "LLM query failed: %s", e.what());
    result->success = false;
    result->message = std::string("LLM query failed: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "LLM raw response: %s", llm_json.c_str());

  // ── Step 2: parse JSON → TaskCommand ────────────────────────────────────────
  feedback->status = "Parsing LLM response...";
  goal_handle->publish_feedback(feedback);

  amm_msgs::msg::TaskCommand task_cmd;
  try {
    task_cmd = parse_llm_response(llm_json);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse LLM response: %s", e.what());
    result->success = false;
    result->message = std::string("Parse error: ") + e.what();
    goal_handle->abort(result);
    return;
  }

  task_cmd.header.stamp    = now();
  task_cmd.header.frame_id = "map";

  // ── Step 3: publish and return ──────────────────────────────────────────────
  task_cmd_pub_->publish(task_cmd);

  result->success      = true;
  result->message      = "Task command parsed successfully.";
  result->task_command = task_cmd;

  RCLCPP_INFO(get_logger(),
    "Parsed: object='%s', room=%d, dest='%s', action='%s'",
    task_cmd.object_name.c_str(),
    task_cmd.source_room,
    task_cmd.destination.c_str(),
    task_cmd.action_type.c_str());

  goal_handle->succeed(result);
}

// ── LLM helpers ───────────────────────────────────────────────────────────────

std::string TaskPlannerNode::build_full_prompt(const std::string & user_prompt) const
{
  // The system instruction is embedded in the prompt because Ollama's
  // /api/generate endpoint uses a single "prompt" field.
  // A chat-style endpoint (/api/chat) can separate system/user, but
  // this format works universally across all Ollama models.
  std::ostringstream ss;
  ss << "You are a robot task parser. Given a natural language instruction, "
     << "output ONLY a valid JSON object — no explanation, no markdown, no code fences. "
     << "The JSON must have exactly these fields:\n"
     << "  \"object_name\" (string): the object to manipulate\n"
     << "  \"source_room\" (integer 1-3): the room where the object is located\n"
     << "  \"destination\" (string): where to bring the object, e.g. \"base\"\n"
     << "  \"action_type\" (string): always \"pick_and_place\"\n\n"
     << "Example input:  Pick up the apple in room 3 and bring it to base.\n"
     << "Example output: {\"object_name\":\"apple\",\"source_room\":3,"
     <<                    "\"destination\":\"base\",\"action_type\":\"pick_and_place\"}\n\n"
     << "Now parse this instruction:\n"
     << user_prompt;
  return ss.str();
}

// libcurl write callback — appends received bytes to the buffer string
size_t TaskPlannerNode::write_callback(
  void * contents, size_t size, size_t nmemb, std::string * buffer)
{
  const size_t total = size * nmemb;
  buffer->append(static_cast<char *>(contents), total);
  return total;
}

std::string TaskPlannerNode::query_ollama(const std::string & user_prompt)
{
  CURL * curl = curl_easy_init();
  if (!curl) {
    throw std::runtime_error("Failed to initialise libcurl handle.");
  }

  // Build request URL and JSON body
  const std::string url  = ollama_host_ + "/api/generate";
  const std::string body =
    "{\"model\":\"" + ollama_model_ + "\","
    "\"prompt\":" + "\"" + build_full_prompt(user_prompt) + "\","
    "\"stream\":false}";

  std::string response_buffer;

  struct curl_slist * headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_URL,            url.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &response_buffer);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT,        static_cast<long>(request_timeout_s_));

  CURLcode res = curl_easy_perform(curl);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    throw std::runtime_error(
      std::string("curl_easy_perform failed: ") + curl_easy_strerror(res));
  }

  // Extract the model's text from the Ollama response envelope
  return json_util::extract_response_field(response_buffer);
}

amm_msgs::msg::TaskCommand TaskPlannerNode::parse_llm_response(const std::string & json_str)
{
  if (json_str.empty()) {
    throw std::runtime_error("LLM returned an empty response.");
  }

  amm_msgs::msg::TaskCommand cmd;
  cmd.object_name = json_util::extract_string(json_str, "object_name");
  cmd.source_room = json_util::extract_int(json_str,    "source_room", -1);
  cmd.destination = json_util::extract_string(json_str, "destination");
  cmd.action_type = json_util::extract_string(json_str, "action_type");

  if (cmd.object_name.empty()) {
    throw std::runtime_error("Missing 'object_name' in LLM JSON: " + json_str);
  }
  if (cmd.source_room < 1 || cmd.source_room > 3) {
    throw std::runtime_error("Invalid 'source_room' (must be 1-3): " + json_str);
  }
  if (cmd.destination.empty()) {
    throw std::runtime_error("Missing 'destination' in LLM JSON: " + json_str);
  }

  return cmd;
}

}  // namespace amm_task_planner

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amm_task_planner::TaskPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
