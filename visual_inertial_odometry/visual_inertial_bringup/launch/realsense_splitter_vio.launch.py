import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("visual_inertial_bringup")
    realsense_utils_share = get_package_share_directory("realsense_utils")

    default_vio_params = os.path.join(
        bringup_share, "config", "visual_inertial_params_realsense_splitter.yaml"
    )
    realsense_launch_file = os.path.join(
        realsense_utils_share, "launch", "realsense.launch.py"
    )

    launch_realsense = LaunchConfiguration("launch_realsense")
    camera_serial_numbers = LaunchConfiguration("camera_serial_numbers")
    startup_delay = LaunchConfiguration("startup_delay_s")
    use_tracks_viz = LaunchConfiguration("use_tracks_viz")
    use_path_viz = LaunchConfiguration("use_path_viz")
    launch_mapping_logger = LaunchConfiguration("launch_mapping_logger")
    params_file = LaunchConfiguration("params_file")
    logger_params_file = LaunchConfiguration("logger_params_file")
    operation_mode = LaunchConfiguration("operation_mode")
    localization_tag_map_path = LaunchConfiguration("localization_tag_map_path")
    tag_topic = LaunchConfiguration("tag_topic")
    enable_infra_gain_correction = LaunchConfiguration("enable_infra_gain_correction")
    infra_enable_auto_exposure = LaunchConfiguration("infra_enable_auto_exposure")
    infra_exposure = LaunchConfiguration("infra_exposure")
    infra_gain = LaunchConfiguration("infra_gain")
    publish_optimized_landmarks = LaunchConfiguration("publish_optimized_landmarks")
    lm_fetch_max = LaunchConfiguration("lm_fetch_max")

    def maybe_launch_mapping_logger(context):
        if operation_mode.perform(context) != "mapping":
            return []
        if launch_mapping_logger.perform(context).lower() != "true":
            return []

        params_path = logger_params_file.perform(context)
        if not params_path:
            logger_share = get_package_share_directory("online_mapping_logger")
            params_path = os.path.join(
                logger_share, "config", "online_mapping_logger_params.yaml"
            )

        return [
            Node(
                package="online_mapping_logger",
                executable="online_mapping_logger_node",
                name="online_mapping_logger",
                output="screen",
                parameters=[params_path],
            )
        ]

    def maybe_launch_localization_node(context):
        if operation_mode.perform(context) != "localization":
            return []

        localization_params = [params_file]
        tag_map_path = localization_tag_map_path.perform(context)
        if tag_map_path:
            localization_params.append(
                {
                    "localization_tag_map_path": ParameterValue(tag_map_path, value_type=str),
                }
            )
        tag_topic_value = tag_topic.perform(context)
        if tag_topic_value:
            localization_params.append(
                {
                    "tag_topic": ParameterValue(tag_topic_value, value_type=str),
                }
            )

        return [
            Node(
                package="visual_inertial",
                executable="localization_node",
                name="visual_inertial_localization",
                output="screen",
                parameters=localization_params,
            )
        ]

    def maybe_launch_optimization_node(context):
        optimization_params = [
            params_file,
            {
                "operation_mode": ParameterValue(operation_mode, value_type=str),
            },
        ]

        publish_landmarks_value = publish_optimized_landmarks.perform(context)
        if publish_landmarks_value:
            optimization_params.append(
                {
                    "publish_optimized_landmarks": publish_landmarks_value.lower() == "true",
                }
            )

        lm_fetch_max_value = lm_fetch_max.perform(context)
        if lm_fetch_max_value:
            optimization_params.append(
                {
                    "lm_fetch_max": int(lm_fetch_max_value),
                }
            )

        return [
            Node(
                package="visual_inertial",
                executable="optimization_node",
                name="visual_inertial_optimization",
                output="screen",
                parameters=optimization_params,
            )
        ]

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            "run_standalone": "True",
            "container_name": "realsense_utils_container",
            "num_cameras": "1",
            "use_intra_process_comms": "True",
            "camera_serial_numbers": camera_serial_numbers,
            "auto_retoggle_emitter_on_off": "True",
            "emitter_retoggle_delay_sec": "5.0",
            "enable_infra_gain_correction": enable_infra_gain_correction,
            "infra_enable_auto_exposure": infra_enable_auto_exposure,
            "infra_exposure": infra_exposure,
            "infra_gain": infra_gain,
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
        actions=[
            tracking_node,
            tracks_viz_node,
            path_viz_node,
            OpaqueFunction(function=maybe_launch_localization_node),
            OpaqueFunction(function=maybe_launch_optimization_node),
            OpaqueFunction(function=maybe_launch_mapping_logger),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_realsense", default_value="true"),
            DeclareLaunchArgument("camera_serial_numbers", default_value=""),
            DeclareLaunchArgument("startup_delay_s", default_value="8.0"),
            DeclareLaunchArgument("use_tracks_viz", default_value="false"),
            DeclareLaunchArgument("use_path_viz", default_value="true"),
            DeclareLaunchArgument("launch_mapping_logger", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_vio_params),
            DeclareLaunchArgument("logger_params_file", default_value=""),
            DeclareLaunchArgument("localization_tag_map_path", default_value=""),
            DeclareLaunchArgument("tag_topic", default_value=""),
            DeclareLaunchArgument("enable_infra_gain_correction", default_value="false"),
            DeclareLaunchArgument("infra_enable_auto_exposure", default_value="false"),
            DeclareLaunchArgument("infra_exposure", default_value="12000"),
            DeclareLaunchArgument("infra_gain", default_value="90"),
            DeclareLaunchArgument("publish_optimized_landmarks", default_value=""),
            DeclareLaunchArgument("lm_fetch_max", default_value=""),
            DeclareLaunchArgument("operation_mode", default_value="mapping"),
            realsense_launch,
            launch_vio_nodes,
        ]
    )
