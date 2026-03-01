from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    """Set up action_sequencer node with MoveIt config."""
    arm_model_str = context.launch_configurations["arm_model"]
    moveit_config_package = "melfa_" + arm_model_str + "_moveit_config"

    moveit_config = MoveItConfigsBuilder(
        arm_model_str, package_name=moveit_config_package
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
            {"arm_model": arm_model_str},
        ],
    )
    return [action_sequencer_node]


def generate_launch_description():
    """Generate launch description with arm_model argument."""
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_model",
            default_value="rv7frl",
            description="Robot arm model name (e.g. rv5as, rv7frl)",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
