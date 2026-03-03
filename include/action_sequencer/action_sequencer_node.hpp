#pragma once

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

extern const rclcpp::Logger LOGGER;

namespace teaching_points
{
// Creates a Pose from position and orientation values
inline geometry_msgs::msg::Pose makePose(
  double x, double y, double z,
  double ox, double oy, double oz, double ow)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = ox;
  p.orientation.y = oy;
  p.orientation.z = oz;
  p.orientation.w = ow;
  return p;
}

extern std::map<std::string, geometry_msgs::msg::Pose> pose_map;
} // namespace teaching_points

class ActionSequencer : public rclcpp::Node {
public:
  // Constructor to create the action_sequencer node
  explicit ActionSequencer(const rclcpp::NodeOptions& options);

  // Creates and configures the MoveGroupInterface
  void initMoveGroup();

  // Sets velocity and acceleration scaling factors
  void setSpeedScale(double vel, double acc);

  // Changes the motion planning pipeline and planner algorithm
  //
  // Available pipeline_id / planner_id combinations:
  //
  // 1) pipeline_id = "pilz_industrial_motion_planner"
  //    - planner_id: "PTP"  (point-to-point, default)
  //    - planner_id: "LIN"  (linear Cartesian motion)
  //    - planner_id: "CIRC" (circular Cartesian motion)
  //
  // 2) pipeline_id = "ompl"
  //    - planner_id: "RRTConnectkConfigDefault"  (fast, default for OMPL)
  //    - planner_id: "RRTkConfigDefault"
  //    - planner_id: "RRTstarkConfigDefault"      (asymptotically optimal)
  //    - planner_id: "PRMkConfigDefault"
  //    - planner_id: "PRMstarkConfigDefault"
  //    - planner_id: "ESTkConfigDefault"
  //    - planner_id: "SBLkConfigDefault"
  //    - planner_id: "KPIECEkConfigDefault"
  //    - planner_id: "BKPIECEkConfigDefault"
  //    - planner_id: "LBKPIECEkConfigDefault"
  //    - planner_id: "TRRTkConfigDefault"
  //
  // 3) pipeline_id = "chomp"
  //    - planner_id: "" (CHOMP has no sub-planner selection)
  //
  // 4) pipeline_id = "stomp"
  //    - planner_id: "" (STOMP has no sub-planner selection)
  void setPlanner(const std::string& pipeline_id, const std::string& planner_id);

  // Plans and executes a motion with return value checking
  bool planAndExecute();

  // Plans and executes a motion to the Cartesian pose target
  bool moveToPoseTarget(const geometry_msgs::msg::Pose& target_pose);

  // Plans and executes a motion to the "home" position
  bool moveToHome();

  // Adds a box collision object to the planning scene
  void addCollisionBox(
    const std::string& id,
    const std::vector<double>& dimensions,
    const geometry_msgs::msg::Pose& pose);

  // Executes the motion sequence (called from a dedicated thread)
  void executeActionSequence();

private:
  std::string planning_group_ = "";
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};
