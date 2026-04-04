#include "amm_mission_executive/mission_executive_node.hpp"

#include <chrono>
#include <map>
#include <thread>

using namespace std::chrono_literals;

namespace amm_mission_executive
{

// ── Utilities ──────────────────────────────────────────────────────────────────

std::string to_string(MissionState s)
{
  switch (s) {
    case MissionState::IDLE:              return "IDLE";
    case MissionState::NAVIGATE_TO_ROOM:  return "NAVIGATE_TO_ROOM";
    case MissionState::PERCEIVE_OBJECT:   return "PERCEIVE_OBJECT";
    case MissionState::PICK_OBJECT:       return "PICK_OBJECT";
    case MissionState::NAVIGATE_TO_BASE:  return "NAVIGATE_TO_BASE";
    case MissionState::PLACE_OBJECT:      return "PLACE_OBJECT";
    case MissionState::SUCCESS:           return "SUCCESS";
    case MissionState::FAILED:            return "FAILED";
    default:                              return "UNKNOWN";
  }
}

// ── Constructor ────────────────────────────────────────────────────────────────

MissionExecutiveNode::MissionExecutiveNode(const rclcpp::NodeOptions & options)
: Node("mission_executive_node", options)
{
  // Load room waypoints from parameters
  load_waypoints();

  // Action clients
  nav_client_   = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  pick_client_  = rclcpp_action::create_client<PickObject>(this,    "pick_object");
  place_client_ = rclcpp_action::create_client<PlaceObject>(this,   "place_object");

  // Action server — accepts TaskCommand goals from task_planner or external callers
  task_action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this,
    "run_mission",
    std::bind(&MissionExecutiveNode::handle_goal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&MissionExecutiveNode::handle_cancel,   this, std::placeholders::_1),
    std::bind(&MissionExecutiveNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "MissionExecutiveNode ready.");
}

// ── Action server callbacks ────────────────────────────────────────────────────

rclcpp_action::GoalResponse MissionExecutiveNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ExecuteTask::Goal> goal)
{
  if (current_state_ != MissionState::IDLE) {
    RCLCPP_WARN(get_logger(), "Rejecting goal — mission already in progress (%s).",
      to_string(current_state_).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Accepting goal: \"%s\"", goal->prompt.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionExecutiveNode::handle_cancel(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> /*goal_handle*/)
{
  RCLCPP_WARN(get_logger(), "Mission cancel requested.");
  current_state_ = MissionState::FAILED;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionExecutiveNode::handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> goal_handle)
{
  std::thread{[this, goal_handle]() { run_mission(goal_handle); }}.detach();
}

// ── Mission execution ──────────────────────────────────────────────────────────

void MissionExecutiveNode::run_mission(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTask>> goal_handle)
{
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result   = std::make_shared<ExecuteTask::Result>();

  // The task_command is expected to be embedded in the goal (set by task_planner).
  // For now the goal carries only a text prompt; the task_command is resolved
  // externally. TODO: wire task_planner output directly into this node.
  // For the template, we use the task embedded in active_task_.

  auto publish_feedback = [&](const std::string & status) {
    feedback->status = status;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "[%s] %s",
      to_string(current_state_).c_str(), status.c_str());
  };

  // ── NAVIGATE_TO_ROOM ─────────────────────────────────────────────────────────
  current_state_ = MissionState::NAVIGATE_TO_ROOM;
  publish_feedback("Navigating to room " + std::to_string(active_task_.source_room) + "...");

  if (!navigate_to_room(active_task_.source_room)) {
    current_state_  = MissionState::FAILED;
    result->success = false;
    result->message = "Navigation to room failed.";
    goal_handle->abort(result);
    return;
  }

  // ── PERCEIVE_OBJECT ──────────────────────────────────────────────────────────
  current_state_ = MissionState::PERCEIVE_OBJECT;
  publish_feedback("Looking for '" + active_task_.object_name + "'...");

  if (!perceive_object(active_task_.object_name)) {
    current_state_  = MissionState::FAILED;
    result->success = false;
    result->message = "Object not found: " + active_task_.object_name;
    goal_handle->abort(result);
    return;
  }

  // ── PICK_OBJECT ──────────────────────────────────────────────────────────────
  current_state_ = MissionState::PICK_OBJECT;
  publish_feedback("Picking '" + active_task_.object_name + "'...");

  if (!pick_object()) {
    current_state_  = MissionState::FAILED;
    result->success = false;
    result->message = "Pick failed.";
    goal_handle->abort(result);
    return;
  }

  // ── NAVIGATE_TO_BASE ─────────────────────────────────────────────────────────
  current_state_ = MissionState::NAVIGATE_TO_BASE;
  publish_feedback("Returning to base...");

  if (!navigate_to_base()) {
    current_state_  = MissionState::FAILED;
    result->success = false;
    result->message = "Navigation to base failed.";
    goal_handle->abort(result);
    return;
  }

  // ── PLACE_OBJECT ─────────────────────────────────────────────────────────────
  current_state_ = MissionState::PLACE_OBJECT;
  publish_feedback("Placing object at base...");

  if (!place_object()) {
    current_state_  = MissionState::FAILED;
    result->success = false;
    result->message = "Place failed.";
    goal_handle->abort(result);
    return;
  }

  // ── SUCCESS ──────────────────────────────────────────────────────────────────
  current_state_  = MissionState::SUCCESS;
  result->success = true;
  result->message = "Mission complete.";
  goal_handle->succeed(result);

  current_state_ = MissionState::IDLE;
  RCLCPP_INFO(get_logger(), "Mission complete.");
}

// ── Step helpers ───────────────────────────────────────────────────────────────

bool MissionExecutiveNode::navigate_to_room(int room_id)
{
  auto it = room_waypoints_.find(room_id);
  if (it == room_waypoints_.end()) {
    RCLCPP_ERROR(get_logger(), "No waypoint configured for room %d.", room_id);
    return false;
  }
  return navigate_to_pose(it->second);
}

bool MissionExecutiveNode::navigate_to_base()
{
  return navigate_to_pose(base_pose_);
}

bool MissionExecutiveNode::perceive_object(const std::string & /*object_name*/)
{
  // TODO: send a goal to the ObjectDetectorNode action server
  //       (amm_perception/DetectObject action — to be defined)
  // For the template this is a stub that always succeeds.
  RCLCPP_INFO(get_logger(), "[STUB] perceive_object — implement with isaac_ros_grounding_dino.");
  return true;
}

bool MissionExecutiveNode::pick_object()
{
  if (!pick_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "PickObject action server not available.");
    return false;
  }

  auto goal = PickObject::Goal{};
  goal.object = detected_object_;

  auto send_goal_options = rclcpp_action::Client<PickObject>::SendGoalOptions{};
  auto future = pick_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send pick goal.");
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Pick goal was rejected.");
    return false;
  }

  auto result_future = pick_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Pick action did not complete.");
    return false;
  }

  return result_future.get().result->success;
}

bool MissionExecutiveNode::place_object()
{
  if (!place_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "PlaceObject action server not available.");
    return false;
  }

  auto goal = PlaceObject::Goal{};
  goal.target_pose = base_pose_;

  auto send_goal_options = rclcpp_action::Client<PlaceObject>::SendGoalOptions{};
  auto future = place_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send place goal.");
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Place goal was rejected.");
    return false;
  }

  auto result_future = place_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Place action did not complete.");
    return false;
  }

  return result_future.get().result->success;
}

bool MissionExecutiveNode::navigate_to_pose(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available.");
    return false;
  }

  auto goal = NavigateToPose::Goal{};
  goal.pose = target_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
  auto future = nav_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send navigation goal.");
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Navigation goal was rejected by Nav2.");
    return false;
  }

  auto result_future = nav_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Navigation action did not complete.");
    return false;
  }

  const auto code = result_future.get().code;
  return (code == rclcpp_action::ResultCode::SUCCEEDED);
}

// ── Parameter helpers ──────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped MissionExecutiveNode::make_pose(
  double x, double y, double z, double qw) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id       = "map";
  p.pose.position.x       = x;
  p.pose.position.y       = y;
  p.pose.position.z       = z;
  p.pose.orientation.w    = qw;
  p.pose.orientation.x    = 0.0;
  p.pose.orientation.y    = 0.0;
  p.pose.orientation.z    = 0.0;
  return p;
}

void MissionExecutiveNode::load_waypoints()
{
  // Declare parameters with default values (override in config/waypoints.yaml)
  declare_parameter("room1.x", 2.0); declare_parameter("room1.y",  0.0);
  declare_parameter("room2.x", 4.0); declare_parameter("room2.y",  4.0);
  declare_parameter("room3.x", 0.0); declare_parameter("room3.y",  4.0);
  declare_parameter("base.x",  0.0); declare_parameter("base.y",   0.0);

  room_waypoints_[1] = make_pose(
    get_parameter("room1.x").as_double(),
    get_parameter("room1.y").as_double(), 0.0, 1.0);

  room_waypoints_[2] = make_pose(
    get_parameter("room2.x").as_double(),
    get_parameter("room2.y").as_double(), 0.0, 1.0);

  room_waypoints_[3] = make_pose(
    get_parameter("room3.x").as_double(),
    get_parameter("room3.y").as_double(), 0.0, 1.0);

  base_pose_ = make_pose(
    get_parameter("base.x").as_double(),
    get_parameter("base.y").as_double(), 0.0, 1.0);

  RCLCPP_INFO(get_logger(), "Loaded waypoints for rooms 1-3 and base.");
}

}  // namespace amm_mission_executive

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amm_mission_executive::MissionExecutiveNode>());
  rclcpp::shutdown();
  return 0;
}
