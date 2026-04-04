#include "amm_navigation_manager/navigation_manager_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace amm_navigation_manager
{

NavigationManagerNode::NavigationManagerNode(const rclcpp::NodeOptions & options)
: Node("navigation_manager_node", options)
{
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  load_waypoints();

  RCLCPP_INFO(get_logger(),
    "NavigationManagerNode ready. %zu waypoints loaded.",
    waypoints_.size());
}

// ── Public API ─────────────────────────────────────────────────────────────────

bool NavigationManagerNode::navigate_to(const std::string & waypoint_name)
{
  auto it = waypoints_.find(waypoint_name);
  if (it == waypoints_.end()) {
    RCLCPP_ERROR(get_logger(), "Unknown waypoint: '%s'", waypoint_name.c_str());
    return false;
  }
  return navigate_to_pose(it->second);
}

bool NavigationManagerNode::navigate_to_pose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available.");
    return false;
  }

  auto goal = NavigateToPose::Goal{};
  goal.pose = pose;

  RCLCPP_INFO(get_logger(),
    "Sending nav goal: (%.2f, %.2f) frame='%s'",
    pose.pose.position.x, pose.pose.position.y,
    pose.header.frame_id.c_str());

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};

  send_options.feedback_callback =
    [this](auto /*gh*/,
           const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      RCLCPP_DEBUG(get_logger(),
        "Nav2 distance remaining: %.2f m", fb->distance_remaining);
    };

  auto future = nav_client_->async_send_goal(goal, send_options);

  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send navigation goal.");
    return false;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Navigation goal rejected by Nav2.");
    return false;
  }

  auto result_future = nav_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Navigation did not complete.");
    return false;
  }

  const bool ok = (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
  RCLCPP_INFO(get_logger(), "Navigation %s.", ok ? "succeeded" : "FAILED");
  return ok;
}

// ── Parameter helpers ──────────────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped NavigationManagerNode::make_pose(
  double x, double y, double yaw_deg) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id    = "map";
  p.pose.position.x    = x;
  p.pose.position.y    = y;
  p.pose.position.z    = 0.0;
  // Convert yaw (degrees) to quaternion (rotation around Z)
  const double yaw = yaw_deg * M_PI / 180.0;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

void NavigationManagerNode::load_waypoints()
{
  // Declare + read parameters; defaults match the Isaac Sim room layout
  declare_parameter("waypoints.room1.x",   2.0);
  declare_parameter("waypoints.room1.y",   0.0);
  declare_parameter("waypoints.room1.yaw", 0.0);

  declare_parameter("waypoints.room2.x",   4.0);
  declare_parameter("waypoints.room2.y",   4.0);
  declare_parameter("waypoints.room2.yaw", 180.0);

  declare_parameter("waypoints.room3.x",   0.0);
  declare_parameter("waypoints.room3.y",   4.0);
  declare_parameter("waypoints.room3.yaw", 90.0);

  declare_parameter("waypoints.base.x",    0.0);
  declare_parameter("waypoints.base.y",    0.0);
  declare_parameter("waypoints.base.yaw",  0.0);

  for (const auto & name : {"room1", "room2", "room3", "base"}) {
    const std::string prefix = "waypoints." + std::string(name);
    waypoints_[name] = make_pose(
      get_parameter(prefix + ".x").as_double(),
      get_parameter(prefix + ".y").as_double(),
      get_parameter(prefix + ".yaw").as_double());
  }
}

}  // namespace amm_navigation_manager

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amm_navigation_manager::NavigationManagerNode>());
  rclcpp::shutdown();
  return 0;
}
