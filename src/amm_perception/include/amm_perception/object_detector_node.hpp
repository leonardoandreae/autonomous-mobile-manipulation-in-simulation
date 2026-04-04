#pragma once

#include <memory>
#include <optional>
#include <string>

#include "amm_msgs/msg/detected_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace amm_perception
{

/**
 * ObjectDetectorNode
 * ==================
 * Detects objects by name and returns their 6-DoF pose.
 *
 * Two backends are supported, selected at runtime via the `backend` parameter:
 *
 *   "gazebo"    (default) — Reads model poses directly from the Gazebo world
 *               via TF frames published by ros_gz_bridge.  CPU-only, no GPU
 *               required.  Object name must match the Gazebo model name
 *               (e.g. "apple", "mug", "bottle").
 *
 *   "isaac_sim" — Uses isaac_ros_grounding_dino (open-vocabulary 2-D detection)
 *               + isaac_ros_foundationpose (6-DoF pose estimation).
 *               Requires NVIDIA GPU and isaac_ros packages.
 */
class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * Trigger a detection cycle for the given object class label.
   * Blocks until a detection is found or the timeout expires.
   *
   * @param object_name  Name/label to search for (e.g. "apple")
   * @param timeout_s    Maximum wait time in seconds
   * @return             Detected object with pose, or nullopt on failure
   */
  std::optional<amm_msgs::msg::DetectedObject> detect(
    const std::string & object_name, double timeout_s = 10.0);

private:
  std::string backend_;           // "gazebo" or "isaac_sim"
  std::string gz_world_name_;     // Gazebo world name (backend=gazebo only)

  // ── TF (Gazebo backend) ───────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;

  // ── Subscriptions (Isaac Sim backend) ────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           rgb_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_sub_;

  // ── Publisher ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<amm_msgs::msg::DetectedObject>::SharedPtr detected_obj_pub_;

  // ── State (Isaac Sim backend) ─────────────────────────────────────────────
  std::string target_label_;
  std::optional<amm_msgs::msg::DetectedObject> latest_detection_;

  // ── Backend implementations ───────────────────────────────────────────────
  std::optional<amm_msgs::msg::DetectedObject> detect_gazebo(
    const std::string & object_name, double timeout_s);

  std::optional<amm_msgs::msg::DetectedObject> detect_isaac(
    const std::string & object_name, double timeout_s);

  // ── Callbacks (Isaac Sim backend) ─────────────────────────────────────────
  void on_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace amm_perception
