import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    spawn_arm_and_hand = LaunchConfiguration("spawn_arm_and_hand")
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )
    
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    # Load controllers (spawn via controller_manager spawner)
    # Always spawn joint_state_broadcaster; arm/hand spawners are conditional.
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(spawn_arm_and_hand),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda_hand_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(spawn_arm_and_hand),
        ),
    ]
    # Delay spawners slightly so controller_manager is ready
    delayed_controller_spawners = TimerAction(period=2.0, actions=controller_spawners)

    # MTC Demo node
    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription(
        [
            # If your ros2_controllers.yaml auto-activates arm/hand controllers,
            # leave spawn_arm_and_hand as 'false' to avoid duplication.
            DeclareLaunchArgument(
                "spawn_arm_and_hand",
                default_value="false",
                description=(
                    "Spawn arm/hand controllers in addition to joint_state_broadcaster. "
                    "Set true if controllers are NOT auto-loaded/activated by your ros2_controllers.yaml."
                ),
            ),
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mtc_node,
            delayed_controller_spawners,
        ]
    )
