#pragma once

#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amm_navigation_manager
{

class NavigationManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit NavigationManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * Navigate to a named waypoint ("room1", "room2", "room3", "base").
   * Blocks until Nav2 reports success or failure.
   * @return true on success.
   */
  bool navigate_to(const std::string & waypoint_name);

  /**
   * Navigate to an arbitrary pose.
   * @return true on success.
   */
  bool navigate_to_pose(const geometry_msgs::msg::PoseStamped & pose);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Named waypoints loaded from parameters
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  void load_waypoints();

  geometry_msgs::msg::PoseStamped make_pose(
    double x, double y, double yaw_deg) const;
};

}  // namespace amm_navigation_manager
