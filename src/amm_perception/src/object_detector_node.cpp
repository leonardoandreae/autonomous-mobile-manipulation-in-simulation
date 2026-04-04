#include "amm_perception/object_detector_node.hpp"

#include <chrono>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace amm_perception
{

// ── Constructor ────────────────────────────────────────────────────────────────

ObjectDetectorNode::ObjectDetectorNode(const rclcpp::NodeOptions & options)
: Node("object_detector_node", options)
{
  // ── Parameters ───────────────────────────────────────────────────────────────
  declare_parameter("backend",        "gazebo");   // "gazebo" | "isaac_sim"
  declare_parameter("gz_world_name",  "three_room_world");
  // Isaac Sim topic parameters (ignored when backend=gazebo)
  declare_parameter("rgb_topic",         "/camera/image_raw");
  declare_parameter("detections_topic",  "/detections");
  declare_parameter("pose_topic",        "/object_poses");
  declare_parameter("confidence_threshold", 0.5);

  backend_       = get_parameter("backend").as_string();
  gz_world_name_ = get_parameter("gz_world_name").as_string();

  // ── Publisher (common) ───────────────────────────────────────────────────────
  detected_obj_pub_ = create_publisher<amm_msgs::msg::DetectedObject>(
    "detected_object", rclcpp::QoS(10));

  if (backend_ == "gazebo") {
    // ── Gazebo backend — TF listener ──────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    RCLCPP_INFO(get_logger(),
      "ObjectDetectorNode ready [backend=gazebo, world=%s]. "
      "Object names must match Gazebo model names.",
      gz_world_name_.c_str());

  } else {
    // ── Isaac Sim backend — vision pipeline subscribers ───────────────────────
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("rgb_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr /*msg*/) {
        // Raw image forwarded to isaac_ros_grounding_dino externally.
      });

    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      get_parameter("detections_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectorNode::on_detections, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("pose_topic").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectorNode::on_pose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "ObjectDetectorNode ready [backend=isaac_sim]. "
      "Detection pipeline: grounding_dino → %s → foundationpose → %s",
      get_parameter("detections_topic").as_string().c_str(),
      get_parameter("pose_topic").as_string().c_str());
  }
}

// ── Public API ─────────────────────────────────────────────────────────────────

std::optional<amm_msgs::msg::DetectedObject> ObjectDetectorNode::detect(
  const std::string & object_name, double timeout_s)
{
  if (backend_ == "gazebo") {
    return detect_gazebo(object_name, timeout_s);
  } else {
    return detect_isaac(object_name, timeout_s);
  }
}

// ── Gazebo backend ─────────────────────────────────────────────────────────────

std::optional<amm_msgs::msg::DetectedObject> ObjectDetectorNode::detect_gazebo(
  const std::string & object_name, double timeout_s)
{
  RCLCPP_INFO(get_logger(),
    "[gazebo] Looking up pose of model '%s' (timeout=%.1fs)...",
    object_name.c_str(), timeout_s);

  // Gazebo publishes each model as a TF frame via ros_gz_bridge
  // (topic: /gz/world_poses, bridged from /world/<world>/pose/info).
  // The model name is used directly as the TF frame_id.
  const std::string target_frame = object_name;
  const std::string source_frame = "world";

  const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);

  while (rclcpp::ok() && now() < deadline) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        source_frame, target_frame, tf2::TimePointZero);

      amm_msgs::msg::DetectedObject obj;
      obj.header.stamp    = tf.header.stamp;
      obj.header.frame_id = source_frame;
      obj.name            = object_name;
      obj.confidence      = 1.0f;  // ground-truth pose from simulator

      obj.pose.header     = obj.header;
      obj.pose.pose.position.x    = tf.transform.translation.x;
      obj.pose.pose.position.y    = tf.transform.translation.y;
      obj.pose.pose.position.z    = tf.transform.translation.z;
      obj.pose.pose.orientation   = tf.transform.rotation;

      RCLCPP_INFO(get_logger(),
        "[gazebo] Found '%s' at (%.3f, %.3f, %.3f)",
        object_name.c_str(),
        obj.pose.pose.position.x,
        obj.pose.pose.position.y,
        obj.pose.pose.position.z);

      detected_obj_pub_->publish(obj);
      return obj;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup pending: %s", ex.what());
      rclcpp::spin_some(get_node_base_interface());
      std::this_thread::sleep_for(50ms);
    }
  }

  RCLCPP_WARN(get_logger(),
    "[gazebo] Model '%s' not found in TF tree within timeout. "
    "Check that the model name matches a Gazebo model in world '%s'.",
    object_name.c_str(), gz_world_name_.c_str());
  return std::nullopt;
}

// ── Isaac Sim backend ──────────────────────────────────────────────────────────

std::optional<amm_msgs::msg::DetectedObject> ObjectDetectorNode::detect_isaac(
  const std::string & object_name, double timeout_s)
{
  RCLCPP_INFO(get_logger(),
    "[isaac_sim] Searching for '%s' (timeout=%.1fs)...",
    object_name.c_str(), timeout_s);

  target_label_     = object_name;
  latest_detection_ = std::nullopt;

  // TODO: publish the target label to isaac_ros_grounding_dino's query topic.
  // e.g.:
  //   auto msg = std_msgs::msg::String();
  //   msg.data = object_name;
  //   grounding_dino_query_pub_->publish(msg);

  const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);

  while (rclcpp::ok() && now() < deadline) {
    rclcpp::spin_some(get_node_base_interface());
    if (latest_detection_.has_value()) {
      RCLCPP_INFO(get_logger(),
        "[isaac_sim] Found '%s' at (%.3f, %.3f, %.3f)",
        object_name.c_str(),
        latest_detection_->pose.pose.position.x,
        latest_detection_->pose.pose.position.y,
        latest_detection_->pose.pose.position.z);
      return latest_detection_;
    }
    std::this_thread::sleep_for(50ms);
  }

  RCLCPP_WARN(get_logger(),
    "[isaac_sim] Object '%s' not found within timeout.", object_name.c_str());
  return std::nullopt;
}

// ── Isaac Sim callbacks ────────────────────────────────────────────────────────

void ObjectDetectorNode::on_detections(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (target_label_.empty()) {
    return;
  }

  const double threshold = get_parameter("confidence_threshold").as_double();

  for (const auto & det : msg->detections) {
    for (const auto & hyp : det.results) {
      if (hyp.hypothesis.class_id == target_label_ &&
          hyp.hypothesis.score >= threshold)
      {
        RCLCPP_DEBUG(get_logger(),
          "[isaac_sim] 2-D detection: '%s' score=%.2f",
          target_label_.c_str(), hyp.hypothesis.score);
        // TODO: forward bounding box to isaac_ros_foundationpose
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
  obj.confidence = 1.0f;

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
