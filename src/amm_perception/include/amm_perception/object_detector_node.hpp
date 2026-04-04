#pragma once

#include <memory>
#include <string>
#include <optional>

#include "amm_msgs/msg/detected_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace amm_perception
{

class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * Trigger a detection cycle for the given object class label.
   * Blocks until a detection is found or the timeout expires.
   *
   * Integration path:
   *   1. Publish the query label to isaac_ros_grounding_dino
   *   2. Receive bounding box detections on /detections
   *   3. Pass best detection to isaac_ros_foundationpose
   *   4. Receive 6-DoF pose on /object_poses
   *
   * @param object_name  Class label to search for (e.g. "apple")
   * @param timeout_s    Maximum wait time in seconds
   * @return             Detected object with pose, or nullopt on failure
   */
  std::optional<amm_msgs::msg::DetectedObject> detect(
    const std::string & object_name, double timeout_s = 10.0);

private:
  // ── Subscriptions ────────────────────────────────────────────────────────────
  // Color image from the robot's wrist/head camera (Isaac Sim publishes this)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Detections from isaac_ros_grounding_dino (open-vocabulary 2-D bounding boxes)
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;

  // 6-DoF poses from isaac_ros_foundationpose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // ── Publishers ───────────────────────────────────────────────────────────────
  // Forward detections for downstream consumption
  rclcpp::Publisher<amm_msgs::msg::DetectedObject>::SharedPtr detected_obj_pub_;

  // ── State ────────────────────────────────────────────────────────────────────
  std::string target_label_;
  std::optional<amm_msgs::msg::DetectedObject> latest_detection_;

  // ── Callbacks ────────────────────────────────────────────────────────────────
  void on_detections(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace amm_perception
