#pragma once

#include <curl/curl.h>
#include <memory>
#include <string>

#include "amm_msgs/action/execute_task.hpp"
#include "amm_msgs/msg/task_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amm_task_planner
{

class TaskPlannerNode : public rclcpp::Node
{
public:
  using ExecuteTask = amm_msgs::action::ExecuteTask;
  using GoalHandle  = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  explicit TaskPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskPlannerNode();

private:
  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Publisher<amm_msgs::msg::TaskCommand>::SharedPtr task_cmd_pub_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string ollama_host_;   // default: "http://localhost:11434"
  std::string ollama_model_;  // default: "llama3.2"
  int         request_timeout_s_;

  // ── Action server callbacks ──────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTask::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);

  void execute(std::shared_ptr<GoalHandle> goal_handle);

  // ── LLM helpers ─────────────────────────────────────────────────────────────

  /**
   * Send a prompt to the local Ollama instance and return the raw response text.
   * @throws std::runtime_error on network / curl failure.
   */
  std::string query_ollama(const std::string & user_prompt);

  /**
   * Parse the JSON string returned by the LLM into a TaskCommand message.
   * Expected JSON schema:
   * {
   *   "object_name": "apple",
   *   "source_room": 3,
   *   "destination": "base",
   *   "action_type": "pick_and_place"
   * }
   */
  amm_msgs::msg::TaskCommand parse_llm_response(const std::string & json_str);

  /**
   * Wrap the user prompt in a system prompt that instructs the LLM
   * to return only valid JSON matching the TaskCommand schema.
   */
  std::string build_full_prompt(const std::string & user_prompt) const;

  // ── libcurl helpers ──────────────────────────────────────────────────────────
  static size_t write_callback(
    void * contents, size_t size, size_t nmemb, std::string * buffer);
};

}  // namespace amm_task_planner
