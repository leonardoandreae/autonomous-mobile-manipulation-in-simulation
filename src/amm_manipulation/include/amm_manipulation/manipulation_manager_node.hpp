#pragma once

#include <memory>
#include <string>

#include "amm_msgs/action/pick_object.hpp"
#include "amm_msgs/action/place_object.hpp"
#include "amm_msgs/msg/detected_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/task_constructor/task.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amm_manipulation
{

class ManipulationManagerNode : public rclcpp::Node
{
public:
  using PickObject  = amm_msgs::action::PickObject;
  using PlaceObject = amm_msgs::action::PlaceObject;

  explicit ManipulationManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Action servers ───────────────────────────────────────────────────────────
  rclcpp_action::Server<PickObject>::SharedPtr  pick_action_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_action_server_;

  // ── MoveIt 2 interfaces ──────────────────────────────────────────────────────
  // Initialised lazily after the node is fully constructed (requires spin).
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  // ── Parameters ───────────────────────────────────────────────────────────────
  std::string arm_group_name_;        // e.g. "fr3_arm"
  std::string gripper_group_name_;    // e.g. "fr3_hand"
  std::string end_effector_link_;     // e.g. "fr3_hand_tcp"
  double      approach_distance_m_;
  double      lift_distance_m_;

  // ── Lazy initialisation ──────────────────────────────────────────────────────
  void init_moveit();
  bool moveit_ready_{false};

  // ── Pick action server ───────────────────────────────────────────────────────
  rclcpp_action::GoalResponse  pick_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickObject::Goal>);

  rclcpp_action::CancelResponse pick_handle_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>>);

  void pick_handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>>);

  void execute_pick(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>>);

  // ── Place action server ──────────────────────────────────────────────────────
  rclcpp_action::GoalResponse  place_handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlaceObject::Goal>);

  rclcpp_action::CancelResponse place_handle_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>>);

  void place_handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>>);

  void execute_place(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>>);

  // ── MoveIt Task Constructor helpers ─────────────────────────────────────────
  /**
   * Build and execute a pick task using MTC stages:
   *   1. Current state
   *   2. Open gripper
   *   3. Move to pre-grasp (above object)
   *   4. Approach (linear move down)
   *   5. Close gripper
   *   6. Lift (linear move up)
   *
   * @param object_pose  Grasp target in the planning frame
   * @param object_id    Collision object ID in the planning scene
   */
  bool run_pick_task(
    const geometry_msgs::msg::PoseStamped & object_pose,
    const std::string & object_id);

  /**
   * Build and execute a place task using MTC stages:
   *   1. Current state (holding object)
   *   2. Move to pre-place (above target)
   *   3. Lower (linear move down)
   *   4. Open gripper
   *   5. Retreat (linear move up)
   */
  bool run_place_task(const geometry_msgs::msg::PoseStamped & target_pose);

  // ── Gripper helpers ──────────────────────────────────────────────────────────
  bool open_gripper();
  bool close_gripper();

  // ── Planning scene helpers ───────────────────────────────────────────────────
  void add_object_to_scene(
    const std::string & id,
    const geometry_msgs::msg::PoseStamped & pose,
    double size_m = 0.05);

  void remove_object_from_scene(const std::string & id);
};

}  // namespace amm_manipulation
