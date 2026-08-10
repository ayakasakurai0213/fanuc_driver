import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    use_mock = LaunchConfiguration("use_mock")
    gpio_config_package = LaunchConfiguration("gpio_config_package")
    gpio_config_path = LaunchConfiguration("gpio_config_path")
    motion_control = LaunchConfiguration("motion_control")

    nodes_to_launch = []

    # Conditionally include the appropriate control launch file
    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "robot_ip": robot_ip,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
            "use_mock": use_mock,
            "motion_control": motion_control,
        }.items(),
        condition=UnlessCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_control)

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_config_package": gpio_config_package,
            "gpio_config_path": gpio_config_path,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
        }.items(),
        condition=IfCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control)

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "use_mock": use_mock.perform(context),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        f"{robot_model.perform(context)}.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_model.perform(context), package_name="fanuc_moveit_config"
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"srdf/{robot_model.perform(context)}.srdf"
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_path = os.path.join(
        get_package_share_directory("fanuc_moveit_config"),
        "config",
        "servo.yaml",
    )
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml(servo_path)
        .to_dict()
    }
    
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "crx5ia_2f_85gripper"}

    # RViz
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("fanuc_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        arguments=["--display-config", rviz_file],
    )
    nodes_to_launch.append(rviz_node)
    
    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )
    nodes_to_launch.append(container)
    
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )
    nodes_to_launch.append(servo_node)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
            choices=[
                "crx3ia",
                "crx5ia",
                "crx10ia",
                "crx10ia_l",
                "crx20ia_l",
                "crx30ia",
                "crx5ia_2f_85gripper", 
            ],
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_config_package",
            default_value="fanuc_hardware_interface",
            description="The package name where gpio_configuration file exists",
        ),
        DeclareLaunchArgument(
            "gpio_config_path",
            default_value="config/example_gpio_config.yaml",
            description="The gpio_configuration file path in gpio_config_package",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "motion_control",
            default_value="1",
            description="Initial motion control state.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
