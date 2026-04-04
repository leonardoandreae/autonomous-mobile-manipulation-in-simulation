#pragma once

#include <map>
#include <memory>
#include <string>

#include "amm_msgs/action/execute_task.hpp"
#include "amm_msgs/action/pick_object.hpp"
#include "amm_msgs/action/place_object.hpp"
#include "amm_msgs/msg/detected_object.hpp"
#include "amm_msgs/msg/task_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amm_mission_manager
{

// ── State machine states ───────────────────────────────────────────────────────
enum class MissionState
{
  IDLE,
  NAVIGATE_TO_ROOM,
  PERCEIVE_OBJECT,
  PICK_OBJECT,
  NAVIGATE_TO_BASE,
  PLACE_OBJECT,
  SUCCESS,
  FAILED
};

std::string to_string(MissionState s);

// ── Node ───────────────────────────────────────────────────────────────────────

class MissionManagerNode : public rclcpp::Node
{
public:
  using ExecuteTask    = amm_msgs::action::ExecuteTask;
  using PickObject     = amm_msgs::action::PickObject;
  using PlaceObject    = amm_msgs::action::PlaceObject;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Action server (entry point) ──────────────────────────────────────────────
  rclcpp_action::Server<ExecuteTask>::SharedPtr task_action_server_;

  // ── Action clients ───────────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<PickObject>::SharedPtr     pick_client_;
  rclcpp_action::Client<PlaceObject>::SharedPtr    place_client_;

  // ── State ────────────────────────────────────────────────────────────────────
  MissionState current_state_{MissionState::IDLE};
  amm_msgs::msg::TaskCommand active_task_;
  amm_msgs::msg::DetectedObject detected_object_;

  // ── Room waypoints (loaded from params) ─────────────────────────────────────
  std::map<int, geometry_msgs::msg::PoseStamped> room_waypoints_;
  geometry_msgs::msg::PoseStamped                base_pose_;

  // ── Task action server callbacks ─────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteTask::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> goal_handle);

  void handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> goal_handle);

  // ── Mission execution (runs in a separate thread) ────────────────────────────
  void run_mission(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> goal_handle);

  // ── Step helpers ─────────────────────────────────────────────────────────────
  bool navigate_to_room(int room_id);
  bool navigate_to_base();
  bool perceive_object(const std::string & object_name);
  bool pick_object();
  bool place_object();

  // ── Navigation helper ────────────────────────────────────────────────────────
  bool navigate_to_pose(const geometry_msgs::msg::PoseStamped & target_pose);

  // ── Param loading helpers ────────────────────────────────────────────────────
  void load_waypoints();
  geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double z, double qw) const;
};

}  // namespace amm_mission_manager
