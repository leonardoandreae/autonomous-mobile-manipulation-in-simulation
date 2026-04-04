#pragma once

#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace amm_slam_manager
{

/**
 * Manages the lifecycle of slam_toolbox and exposes services for:
 *   - switching to mapping mode (async SLAM)
 *   - switching to localisation mode (static map + AMCL-style)
 *   - saving the current map
 */
class SlamManagerNode : public rclcpp::Node
{
public:
  explicit SlamManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Service servers (for external callers) ───────────────────────────────────
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_localization_srv_;

  // ── Service clients (to control slam_toolbox lifecycle node) ────────────────
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    get_state_client_;

  // ── slam_toolbox serialization service ───────────────────────────────────────
  // slam_toolbox exposes /slam_toolbox/save_map and /slam_toolbox/serialize_map
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_map_client_;

  // ── Parameters ───────────────────────────────────────────────────────────────
  std::string slam_node_name_;  // e.g. "slam_toolbox"
  std::string map_save_dir_;    // directory to store saved maps

  // ── Service handlers ─────────────────────────────────────────────────────────
  void handle_start_mapping(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  req,
    std::shared_ptr<std_srvs::srv::Empty::Response>       res);

  void handle_save_map(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  req,
    std::shared_ptr<std_srvs::srv::Empty::Response>       res);

  void handle_start_localization(
    const std::shared_ptr<std_srvs::srv::Empty::Request>  req,
    std::shared_ptr<std_srvs::srv::Empty::Response>       res);

  // ── Lifecycle helpers ────────────────────────────────────────────────────────
  bool transition_slam_toolbox(uint8_t transition_id);
  uint8_t get_slam_toolbox_state();
};

}  // namespace amm_slam_manager
