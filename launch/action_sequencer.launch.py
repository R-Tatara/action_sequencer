from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arm_model = "rv7frl"

    moveit_config = MoveItConfigsBuilder(
        arm_model, package_name="melfa_" + arm_model + "_moveit_config"
    ).to_moveit_configs()

    action_sequencer_node = Node(
        package="action_sequencer",
        executable="action_sequencer",
        name="action_sequencer",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"arm_model": arm_model},
        ],
    )

    return LaunchDescription([action_sequencer_node])
