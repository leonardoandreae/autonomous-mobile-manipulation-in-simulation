#include "amm_perception/object_detector_node.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace amm_perception
{

ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions & options)
: Node("object_detector_node", options)
{
  // ── Parameters ───────────────────────────────────────────────────────────────
  declare_parameter("rgb_topic",         "/rgb/image_raw");
  declare_parameter("camera_info_topic", "/rgb/camera_info");
  declare_parameter("detections_topic",  "/detections");   // from isaac_ros_grounding_dino
  declare_parameter("pose_topic",        "/object_poses"); // from isaac_ros_foundationpose
  declare_parameter("confidence_threshold", 0.5);

  // ── Subscribers ──────────────────────────────────────────────────────────────
  rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("rgb_topic").as_string(),
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr /*msg*/) {
      // Raw image is forwarded to isaac_ros_grounding_dino via the Isaac ROS
      // pipeline — this node only receives the processed results.
    });

  detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    get_parameter("detections_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&ObjectDetectorNode::on_detections, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("pose_topic").as_string(),
    rclcpp::SensorDataQoS(),
    std::bind(&ObjectDetectorNode::on_pose, this, std::placeholders::_1));

  // ── Publisher ────────────────────────────────────────────────────────────────
  detected_obj_pub_ = create_publisher<amm_msgs::msg::DetectedObject>(
    "detected_object", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "ObjectDetectorNode ready.");
  RCLCPP_INFO(get_logger(),
    "Detection pipeline: isaac_ros_grounding_dino → %s → isaac_ros_foundationpose → %s",
    get_parameter("detections_topic").as_string().c_str(),
    get_parameter("pose_topic").as_string().c_str());
}

// ── Public API ─────────────────────────────────────────────────────────────────

std::optional<amm_msgs::msg::DetectedObject> ObjectDetectorNode::detect(
  const std::string & object_name, double timeout_s)
{
  RCLCPP_INFO(get_logger(), "Searching for '%s' (timeout=%.1fs)...", object_name.c_str(), timeout_s);

  target_label_      = object_name;
  latest_detection_  = std::nullopt;

  // TODO: publish the target label to isaac_ros_grounding_dino's query topic
  // so it knows what class to look for. GroundingDINO accepts free-form text.
  // e.g.:
  //   auto msg = std_msgs/msg/String();
  //   msg.data = object_name;
  //   grounding_dino_query_pub_->publish(msg);

  const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);

  while (rclcpp::ok() && now() < deadline) {
    rclcpp::spin_some(get_node_base_interface());
    if (latest_detection_.has_value()) {
      RCLCPP_INFO(get_logger(),
        "Found '%s' at (%.3f, %.3f, %.3f)",
        object_name.c_str(),
        latest_detection_->pose.pose.position.x,
        latest_detection_->pose.pose.position.y,
        latest_detection_->pose.pose.position.z);
      return latest_detection_;
    }
    std::this_thread::sleep_for(50ms);
  }

  RCLCPP_WARN(get_logger(), "Object '%s' not found within timeout.", object_name.c_str());
  return std::nullopt;
}

// ── Callbacks ──────────────────────────────────────────────────────────────────

void ObjectDetectorNode::on_detections(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (target_label_.empty()) {
    return;
  }

  const double threshold =
    get_parameter("confidence_threshold").as_double();

  for (const auto & det : msg->detections) {
    // GroundingDINO sets hypothesis.class_id to the queried text label
    for (const auto & hyp : det.results) {
      if (hyp.hypothesis.class_id == target_label_ &&
          hyp.hypothesis.score >= threshold)
      {
        // Object found in 2-D; wait for pose from FoundationPose
        RCLCPP_DEBUG(get_logger(),
          "2-D detection: '%s' score=%.2f",
          target_label_.c_str(), hyp.hypothesis.score);

        // TODO: forward the 2-D bounding box crop to isaac_ros_foundationpose
        //       so it can estimate the 6-DoF pose.
        break;
      }
    }
  }
}

void ObjectDetectorNode::on_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (target_label_.empty()) {
    return;
  }

  amm_msgs::msg::DetectedObject obj;
  obj.header     = msg->header;
  obj.name       = target_label_;
  obj.pose       = *msg;
  obj.confidence = 1.0f;  // FoundationPose does not return a scalar confidence

  latest_detection_ = obj;
  detected_obj_pub_->publish(obj);
}

}  // namespace amm_perception

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amm_perception::ObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
