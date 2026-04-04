#include "amm_slam_manager/slam_manager_node.hpp"

#include <chrono>

#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

namespace amm_slam_manager
{

SlamManagerNode::SlamManagerNode(const rclcpp::NodeOptions & options)
: Node("slam_manager_node", options)
{
  declare_parameter("slam_node_name", "slam_toolbox");
  declare_parameter("map_save_dir",   "/tmp/amm_maps");

  get_parameter("slam_node_name", slam_node_name_);
  get_parameter("map_save_dir",   map_save_dir_);

  // ── Service servers ──────────────────────────────────────────────────────────
  start_mapping_srv_ = create_service<std_srvs::srv::Empty>(
    "slam/start_mapping",
    std::bind(&SlamManagerNode::handle_start_mapping, this,
              std::placeholders::_1, std::placeholders::_2));

  save_map_srv_ = create_service<std_srvs::srv::Empty>(
    "slam/save_map",
    std::bind(&SlamManagerNode::handle_save_map, this,
              std::placeholders::_1, std::placeholders::_2));

  start_localization_srv_ = create_service<std_srvs::srv::Empty>(
    "slam/start_localization",
    std::bind(&SlamManagerNode::handle_start_localization, this,
              std::placeholders::_1, std::placeholders::_2));

  // ── Lifecycle clients ─────────────────────────────────────────────────────────
  change_state_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
    slam_node_name_ + "/change_state");

  get_state_client_ = create_client<lifecycle_msgs::srv::GetState>(
    slam_node_name_ + "/get_state");

  save_map_client_ = create_client<std_srvs::srv::Empty>(
    "/slam_toolbox/save_map");

  RCLCPP_INFO(get_logger(),
    "SlamManagerNode ready. Managing lifecycle of '%s'.",
    slam_node_name_.c_str());
  RCLCPP_INFO(get_logger(),
    "Services: slam/start_mapping | slam/save_map | slam/start_localization");
}

// ── Service handlers ───────────────────────────────────────────────────────────

void SlamManagerNode::handle_start_mapping(
  const std::shared_ptr<std_srvs::srv::Empty::Request>  /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>       /*res*/)
{
  RCLCPP_INFO(get_logger(), "Starting SLAM mapping mode...");

  // slam_toolbox in async_slam mode continuously builds the map.
  // If the node is unconfigured, activate it. If already active, do nothing.
  const uint8_t state = get_slam_toolbox_state();
  RCLCPP_INFO(get_logger(), "slam_toolbox current state: %u", state);

  // lifecycle_msgs/msg/State: UNCONFIGURED=1, INACTIVE=2, ACTIVE=3
  if (state == 1 /* UNCONFIGURED */) {
    transition_slam_toolbox(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    transition_slam_toolbox(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  } else if (state == 2 /* INACTIVE */) {
    transition_slam_toolbox(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  RCLCPP_INFO(get_logger(), "SLAM mapping active.");
}

void SlamManagerNode::handle_save_map(
  const std::shared_ptr<std_srvs::srv::Empty::Request>  /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>       /*res*/)
{
  RCLCPP_INFO(get_logger(), "Saving map to %s...", map_save_dir_.c_str());

  if (!save_map_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(get_logger(), "slam_toolbox save_map service not available.");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future  = save_map_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Map saved.");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to save map.");
  }
}

void SlamManagerNode::handle_start_localization(
  const std::shared_ptr<std_srvs::srv::Empty::Request>  /*req*/,
  std::shared_ptr<std_srvs::srv::Empty::Response>       /*res*/)
{
  RCLCPP_INFO(get_logger(), "Switching to localisation mode...");
  // In slam_toolbox, switch to localization-only mode by calling the
  // /slam_toolbox/toggle_interactive_mode service or by launching with
  // the localization_slam_toolbox_node.
  // TODO: implement mode switch via slam_toolbox services.
  RCLCPP_INFO(get_logger(), "[STUB] Localization mode — not yet implemented.");
}

// ── Lifecycle helpers ──────────────────────────────────────────────────────────

bool SlamManagerNode::transition_slam_toolbox(uint8_t transition_id)
{
  if (!change_state_client_->wait_for_service(3s)) {
    RCLCPP_ERROR(get_logger(), "slam_toolbox change_state service not available.");
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  auto future = change_state_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to call change_state.");
    return false;
  }

  return future.get()->success;
}

uint8_t SlamManagerNode::get_slam_toolbox_state()
{
  if (!get_state_client_->wait_for_service(3s)) {
    RCLCPP_WARN(get_logger(), "slam_toolbox get_state service not available. Assuming INACTIVE.");
    return 2;
  }

  auto req    = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = get_state_client_->async_send_request(req);

  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future)
      != rclcpp::FutureReturnCode::SUCCESS)
  {
    return 2;
  }

  return future.get()->current_state.id;
}

}  // namespace amm_slam_manager

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amm_slam_manager::SlamManagerNode>());
  rclcpp::shutdown();
  return 0;
}
