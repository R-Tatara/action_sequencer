#include "action_sequencer/action_sequencer_node.hpp"

const rclcpp::Logger LOGGER = rclcpp::get_logger("action_sequencer");

namespace teaching_points
{
std::map<std::string, geometry_msgs::msg::Pose> pose_map = {
  {"pick_approach",  makePose(0.6, -0.2, 0.4, 0.0, 1.0, 0.0, 0.0)},
  {"pick_grasp",     makePose(0.6, -0.2, 0.2, 0.0, 1.0, 0.0, 0.0)},
  {"pick_retreat",   makePose(0.6, -0.2, 0.4, 0.0, 1.0, 0.0, 0.0)},
  {"place_approach", makePose(0.6, 0.2, 0.4, 0.0, 1.0, 0.0, 0.0)},
  {"place_release",  makePose(0.6, 0.2, 0.2, 0.0, 1.0, 0.0, 0.0)},
  {"place_retreat",  makePose(0.6, 0.2, 0.4, 0.0, 1.0, 0.0, 0.0)},
};
} // namespace teaching_points

// Constructor to create the action_sequencer node
ActionSequencer::ActionSequencer(const rclcpp::NodeOptions& options)
  : Node("action_sequencer", options) {
  RCLCPP_INFO(LOGGER, "ActionSequencer node initialized.");
}

// Creates and configures the MoveGroupInterface
void ActionSequencer::initMoveGroup() {
  // Retrieve the arm model from the parameter server
  planning_group_ = get_parameter("arm_model").as_string();

  // Instantiate MoveGroupInterface
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), planning_group_);

  // Set the motion planning planner parameters
  setPlanner("pilz_industrial_motion_planner", "PTP");
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setPoseReferenceFrame("world");
  setSpeedScale(0.1, 0.1);
}

// Sets velocity and acceleration scaling factors
void ActionSequencer::setSpeedScale(double vel, double acc) {
  if (!move_group_) {
    RCLCPP_ERROR(LOGGER, "MoveGroup is not initialized.");
    return;
  }
  move_group_->setMaxVelocityScalingFactor(vel);
  move_group_->setMaxAccelerationScalingFactor(acc);
}

// Changes the motion planning pipeline and planner algorithm
void ActionSequencer::setPlanner(const std::string& pipeline_id,
                                 const std::string& planner_id) {
  if (!move_group_) {
    RCLCPP_ERROR(LOGGER, "MoveGroup is not initialized.");
    return;
  }
  move_group_->setPlanningPipelineId(pipeline_id);
  move_group_->setPlannerId(planner_id);
  RCLCPP_INFO(LOGGER, "Planner set to pipeline='%s', planner='%s'",
    pipeline_id.c_str(), planner_id.c_str());
}

// Plans and executes a motion with return value checking
bool ActionSequencer::planAndExecute() {
  if (!move_group_) {
    RCLCPP_ERROR(LOGGER, "MoveGroup is not initialized.");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group_->plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Planning failed: %d", plan_result.val);
    return false;
  }

  auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Execution failed: %d", exec_result.val);
    return false;
  }

  return true;
}

// Plans and executes a motion to the Cartesian pose target
bool ActionSequencer::moveToPoseTarget(const geometry_msgs::msg::Pose& target_pose) {
  RCLCPP_INFO(LOGGER, "Moving to pose [%.3f, %.3f, %.3f]",
    target_pose.position.x, target_pose.position.y, target_pose.position.z);
  move_group_->setPoseTarget(target_pose);
  return planAndExecute();
}

// Plans and executes a motion to the "home" position
bool ActionSequencer::moveToHome() {
  RCLCPP_INFO(LOGGER, "Moving to home position");
  move_group_->setJointValueTarget(move_group_->getNamedTargetValues("home"));
  return planAndExecute();
}

// Adds a box collision object to the planning scene
void ActionSequencer::addCollisionBox(
    const std::string& id,
    const std::vector<double>& dimensions,
    const geometry_msgs::msg::Pose& pose) {
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = move_group_->getPlanningFrame();
  obj.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(dimensions.begin(), dimensions.end());

  obj.primitives.push_back(primitive);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects({obj});
  RCLCPP_INFO(LOGGER, "Added collision box: %s", id.c_str());
}

// Executes the motion sequence (called from a dedicated thread)
void ActionSequencer::executeActionSequence() {
  const auto & tp = teaching_points::pose_map;
  initMoveGroup();

  // Add box object
  addCollisionBox("desk", {0.4, 0.6, 0.1},
    teaching_points::makePose(0.6, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0));

  // Action sequence
  if (!moveToHome() ||
      !moveToPoseTarget(tp.at("pick_approach")) ||
      !moveToPoseTarget(tp.at("pick_grasp")) ||
      !moveToPoseTarget(tp.at("pick_retreat")) ||
      !moveToPoseTarget(tp.at("place_approach")) ||
      !moveToPoseTarget(tp.at("place_release")) ||
      !moveToPoseTarget(tp.at("place_retreat")) ||
      !moveToHome()) {
    RCLCPP_ERROR(LOGGER, "Action sequence aborted due to motion failure");
    return;
  }
  RCLCPP_INFO(LOGGER, "Action sequence completed successfully");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create the action_sequencer node
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<ActionSequencer>(node_options);

  // Set up a multi-threaded executor for concurrent callback processing
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Run action sequence in a separate thread
  auto motion_thread = std::thread([&node, &executor]() {
    node->executeActionSequence();
    executor.cancel();
  });

  executor.spin();
  motion_thread.join();
  rclcpp::shutdown();
  return 0;
}
