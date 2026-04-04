#include "amm_manipulation/manipulation_manager_node.hpp"

#include <thread>
#include <chrono>

#include "moveit/task_constructor/stages/current_state.h"
#include "moveit/task_constructor/stages/move_to.h"
#include "moveit/task_constructor/stages/move_relative.h"
#include "moveit/task_constructor/stages/modify_planning_scene.h"
#include "moveit/task_constructor/solvers/cartesian_path.h"
#include "moveit/task_constructor/solvers/pipeline_planner.h"
#include "moveit_msgs/msg/collision_object.hpp"

using namespace std::chrono_literals;
namespace mtc = moveit::task_constructor;

namespace amm_manipulation
{

// ── Constructor ────────────────────────────────────────────────────────────────

ManipulationManagerNode::ManipulationManagerNode(const rclcpp::NodeOptions & options)
: Node("manipulation_manager_node", options)
{
  declare_parameter("arm_group_name",      "fr3_arm");
  declare_parameter("gripper_group_name",  "fr3_hand");
  declare_parameter("end_effector_link",   "fr3_hand_tcp");
  declare_parameter("approach_distance_m", 0.10);
  declare_parameter("lift_distance_m",     0.15);

  get_parameter("arm_group_name",      arm_group_name_);
  get_parameter("gripper_group_name",  gripper_group_name_);
  get_parameter("end_effector_link",   end_effector_link_);
  get_parameter("approach_distance_m", approach_distance_m_);
  get_parameter("lift_distance_m",     lift_distance_m_);

  // ── Action servers ───────────────────────────────────────────────────────────
  pick_action_server_ = rclcpp_action::create_server<PickObject>(
    this, "pick_object",
    std::bind(&ManipulationManagerNode::pick_handle_goal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationManagerNode::pick_handle_cancel,   this, std::placeholders::_1),
    std::bind(&ManipulationManagerNode::pick_handle_accepted, this, std::placeholders::_1));

  place_action_server_ = rclcpp_action::create_server<PlaceObject>(
    this, "place_object",
    std::bind(&ManipulationManagerNode::place_handle_goal,     this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationManagerNode::place_handle_cancel,   this, std::placeholders::_1),
    std::bind(&ManipulationManagerNode::place_handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "ManipulationManagerNode ready (MoveIt not yet initialised).");
  RCLCPP_INFO(get_logger(),
    "Arm='%s'  Gripper='%s'  EEF='%s'",
    arm_group_name_.c_str(), gripper_group_name_.c_str(), end_effector_link_.c_str());
}

// ── Lazy MoveIt initialisation ────────────────────────────────────────────────

void ManipulationManagerNode::init_moveit()
{
  if (moveit_ready_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Initialising MoveIt 2 interfaces...");

  arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), arm_group_name_);

  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), gripper_group_name_);

  planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  arm_group_->setPlanningTime(10.0);
  arm_group_->setNumPlanningAttempts(5);
  arm_group_->setMaxVelocityScalingFactor(0.5);
  arm_group_->setMaxAccelerationScalingFactor(0.5);
  arm_group_->setEndEffectorLink(end_effector_link_);

  moveit_ready_ = true;
  RCLCPP_INFO(get_logger(), "MoveIt 2 ready.");
}

// ── Pick action server ─────────────────────────────────────────────────────────

rclcpp_action::GoalResponse ManipulationManagerNode::pick_handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PickObject::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Pick goal received for object: '%s'",
    goal->object.name.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationManagerNode::pick_handle_cancel(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>> /*gh*/)
{
  RCLCPP_WARN(get_logger(), "Pick cancelled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationManagerNode::pick_handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>> gh)
{
  std::thread{[this, gh]() { execute_pick(gh); }}.detach();
}

void ManipulationManagerNode::execute_pick(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PickObject>> gh)
{
  init_moveit();

  auto fb  = std::make_shared<PickObject::Feedback>();
  auto res = std::make_shared<PickObject::Result>();

  const auto & object = gh->get_goal()->object;
  const std::string obj_id = "target_object";

  fb->status   = "Adding object to planning scene...";
  fb->progress = 0.1f;
  gh->publish_feedback(fb);
  add_object_to_scene(obj_id, object.pose, 0.05);

  fb->status   = "Planning and executing pick...";
  fb->progress = 0.3f;
  gh->publish_feedback(fb);

  const bool ok = run_pick_task(object.pose, obj_id);

  res->success = ok;
  res->message = ok ? "Pick succeeded." : "Pick failed — check MTC output.";

  if (ok) {
    fb->progress = 1.0f;
    fb->status   = "Pick complete.";
    gh->publish_feedback(fb);
    gh->succeed(res);
  } else {
    gh->abort(res);
  }
}

// ── Place action server ────────────────────────────────────────────────────────

rclcpp_action::GoalResponse ManipulationManagerNode::place_handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlaceObject::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulationManagerNode::place_handle_cancel(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>> /*gh*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulationManagerNode::place_handle_accepted(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>> gh)
{
  std::thread{[this, gh]() { execute_place(gh); }}.detach();
}

void ManipulationManagerNode::execute_place(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceObject>> gh)
{
  init_moveit();

  auto fb  = std::make_shared<PlaceObject::Feedback>();
  auto res = std::make_shared<PlaceObject::Result>();

  fb->status   = "Planning and executing place...";
  fb->progress = 0.3f;
  gh->publish_feedback(fb);

  const bool ok = run_place_task(gh->get_goal()->target_pose);

  res->success = ok;
  res->message = ok ? "Place succeeded." : "Place failed.";

  if (ok) {
    remove_object_from_scene("target_object");
    gh->succeed(res);
  } else {
    gh->abort(res);
  }
}

// ── MoveIt Task Constructor ────────────────────────────────────────────────────

bool ManipulationManagerNode::run_pick_task(
  const geometry_msgs::msg::PoseStamped & object_pose,
  const std::string & object_id)
{
  // ── Build MTC Task ────────────────────────────────────────────────────────────
  mtc::Task task{"pick_task"};
  task.stages()->setName("pick");
  task.loadRobotModel(shared_from_this());

  const auto cartesian  = std::make_shared<mtc::solvers::CartesianPath>();
  const auto pipeline   = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

  cartesian->setMaxVelocityScalingFactor(0.3);
  cartesian->setMaxAccelerationScalingFactor(0.3);

  // 1. Current state
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stage));
  }

  // 2. Open gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", pipeline);
    stage->setGroup(gripper_group_name_);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // 3. Move arm above object (pre-grasp)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-grasp", pipeline);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::PoseStamped pre_grasp_pose = object_pose;
    pre_grasp_pose.pose.position.z += approach_distance_m_;
    stage->setGoal(pre_grasp_pose);
    task.add(std::move(stage));
  }

  // 4. Approach — linear move down to grasp
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = end_effector_link_;
    dir.vector.z        = approach_distance_m_;  // positive Z = forward for FR3 EEF
    stage->setDirection(dir);
    task.add(std::move(stage));
  }

  // 5. Close gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("grasp", pipeline);
    stage->setGroup(gripper_group_name_);
    stage->setGoal("close");
    task.add(std::move(stage));
  }

  // 6. Attach object to end effector
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object_id, end_effector_link_);
    task.add(std::move(stage));
  }

  // 7. Lift
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z        = lift_distance_m_;
    stage->setDirection(dir);
    task.add(std::move(stage));
  }

  // ── Plan & Execute ────────────────────────────────────────────────────────────
  try {
    task.init();
    if (!task.plan(5)) {
      RCLCPP_ERROR(get_logger(), "MTC pick planning failed.");
      return false;
    }
    task.introspection().publishSolution(*task.solutions().front());

    auto result = task.execute(*task.solutions().front());
    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR(get_logger(), "MTC init error: %s", e.what());
    return false;
  }
}

bool ManipulationManagerNode::run_place_task(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  mtc::Task task{"place_task"};
  task.stages()->setName("place");
  task.loadRobotModel(shared_from_this());

  const auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  const auto pipeline  = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

  cartesian->setMaxVelocityScalingFactor(0.3);

  // 1. Current state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // 2. Move to pre-place (above target)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("pre-place", pipeline);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::PoseStamped pre_place = target_pose;
    pre_place.pose.position.z += approach_distance_m_;
    stage->setGoal(pre_place);
    task.add(std::move(stage));
  }

  // 3. Lower onto surface
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z        = -approach_distance_m_;
    stage->setDirection(dir);
    task.add(std::move(stage));
  }

  // 4. Detach object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("target_object", end_effector_link_);
    task.add(std::move(stage));
  }

  // 5. Open gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", pipeline);
    stage->setGroup(gripper_group_name_);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // 6. Retreat
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
    stage->setGroup(arm_group_name_);
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z        = lift_distance_m_;
    stage->setDirection(dir);
    task.add(std::move(stage));
  }

  try {
    task.init();
    if (!task.plan(5)) {
      RCLCPP_ERROR(get_logger(), "MTC place planning failed.");
      return false;
    }
    auto result = task.execute(*task.solutions().front());
    return (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  } catch (const mtc::InitStageException & e) {
    RCLCPP_ERROR(get_logger(), "MTC place init error: %s", e.what());
    return false;
  }
}

// ── Gripper helpers ────────────────────────────────────────────────────────────

bool ManipulationManagerNode::open_gripper()
{
  gripper_group_->setNamedTarget("open");
  return (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}

bool ManipulationManagerNode::close_gripper()
{
  gripper_group_->setNamedTarget("close");
  return (gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}

// ── Planning scene helpers ─────────────────────────────────────────────────────

void ManipulationManagerNode::add_object_to_scene(
  const std::string & id,
  const geometry_msgs::msg::PoseStamped & pose,
  double size_m)
{
  moveit_msgs::msg::CollisionObject co;
  co.id               = id;
  co.header           = pose.header;
  co.operation        = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type           = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions     = {size_m, size_m, size_m};

  co.primitives.push_back(prim);
  co.primitive_poses.push_back(pose.pose);

  planning_scene_->applyCollisionObject(co);
}

void ManipulationManagerNode::remove_object_from_scene(const std::string & id)
{
  moveit_msgs::msg::CollisionObject co;
  co.id        = id;
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_->applyCollisionObject(co);
}

}  // namespace amm_manipulation

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amm_manipulation::ManipulationManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
