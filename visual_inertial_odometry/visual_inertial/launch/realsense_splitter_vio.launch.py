import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    visual_inertial_share = get_package_share_directory("visual_inertial")
    realsense_utils_share = get_package_share_directory("realsense_utils")

    default_vio_params = os.path.join(
        visual_inertial_share, "config", "visual_inertial_params_realsense_splitter.yaml"
    )
    realsense_launch_file = os.path.join(
        realsense_utils_share, "launch", "realsense.launch.py"
    )

    launch_realsense = LaunchConfiguration("launch_realsense")
    camera_serial_numbers = LaunchConfiguration("camera_serial_numbers")
    startup_delay = LaunchConfiguration("startup_delay_s")
    use_tracks_viz = LaunchConfiguration("use_tracks_viz")
    use_path_viz = LaunchConfiguration("use_path_viz")
    params_file = LaunchConfiguration("params_file")
    operation_mode = LaunchConfiguration("operation_mode")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            "run_standalone": "True",
            "container_name": "realsense_utils_container",
            "num_cameras": "1",
            "use_intra_process_comms" : "True",
            "camera_serial_numbers": camera_serial_numbers,
            "auto_retoggle_emitter_on_off": "True",
            "emitter_retoggle_delay_sec": "5.0",
        }.items(),
        condition=IfCondition(launch_realsense),
    )

    tracking_node = Node(
        package="visual_inertial",
        executable="tracking_node",
        name="visual_inertial",
        output="screen",
        parameters=[params_file],
    )

    optimization_node = Node(
        package="visual_inertial",
        executable="optimization_node",
        name="visual_inertial_optimization",
        output="screen",
        parameters=[
            params_file,
            {
                "operation_mode": ParameterValue(operation_mode, value_type=str),
            },
        ],
    )

    localization_node = Node(
        package="visual_inertial",
        executable="localization_node",
        name="visual_inertial_localization",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(PythonExpression(["'", operation_mode, "' == 'localization'"])),
    )

    tracks_viz_node = Node(
        package="visual_inertial",
        executable="tracks_viz_node",
        name="tracks_viz_node",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(use_tracks_viz),
    )

    path_viz_node = Node(
        package="visual_inertial",
        executable="path_viz_node",
        name="body_path_node",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(use_path_viz),
    )

    launch_vio_nodes = TimerAction(
        period=startup_delay,
        actions=[tracking_node, tracks_viz_node, path_viz_node, localization_node, optimization_node],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_realsense", default_value="true"),
            DeclareLaunchArgument("camera_serial_numbers", default_value=""),
            DeclareLaunchArgument("startup_delay_s", default_value="8.0"),
            DeclareLaunchArgument("use_tracks_viz", default_value="false"),
            DeclareLaunchArgument("use_path_viz", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_vio_params),
            DeclareLaunchArgument("operation_mode", default_value="mapping"),
            realsense_launch,
            launch_vio_nodes,
        ]
    )
