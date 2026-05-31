import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _default_run_splitter_list(num_cameras: int):
    run_splitter_list = [False] * num_cameras
    if num_cameras > 0:
        run_splitter_list[0] = True
    return run_splitter_list


def _load_camera_config(config_file_path: str):
    with open(config_file_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    if not isinstance(config, dict):
        raise RuntimeError(f'RealSense config must be a YAML mapping: {config_file_path}')
    return config


def _camera_node(camera_name: str, config_params, intra_process_comms, serial_number=None):
    parameters = [config_params, {'camera_name': camera_name}]
    if serial_number:
        parameters.append({'serial_no': str(serial_number)})
    return ComposableNode(
        name=camera_name,
        namespace='',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=parameters,
        extra_arguments=[{'use_intra_process_comms': intra_process_comms}],
    )


def _splitter_node(camera_name: str, intra_process_comms, enable_infra_gain_correction):
    remappings = [
        ('input/infra_1', f'/{camera_name}/infra1/image_rect_raw'),
        ('input/infra_1_metadata', f'/{camera_name}/infra1/metadata'),
        ('input/infra_2', f'/{camera_name}/infra2/image_rect_raw'),
        ('input/infra_2_metadata', f'/{camera_name}/infra2/metadata'),
        ('input/depth', f'/{camera_name}/depth/image_rect_raw'),
        ('input/depth_metadata', f'/{camera_name}/depth/metadata'),
        ('input/pointcloud', f'/{camera_name}/depth/color/points'),
        ('input/pointcloud_metadata', f'/{camera_name}/depth/metadata'),
    ]
    if enable_infra_gain_correction:
        remappings.extend([
            ('~/output/infra_1', f'/{camera_name}/realsense_splitter_node/raw/infra_1'),
            ('~/output/infra_2', f'/{camera_name}/realsense_splitter_node/raw/infra_2'),
        ])

    return ComposableNode(
        namespace=camera_name,
        name='realsense_splitter_node',
        package='realsense_utils',
        plugin='realsense_utils::RealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA',
        }],
        remappings=remappings,
        extra_arguments=[{'use_intra_process_comms': intra_process_comms}],
    )


def _infra_gain_correction_node(
    camera_name: str,
    node_name: str,
    intra_process_comms,
    input_topic: str,
    output_topic: str,
    infra_gain_map_path,
    resize_gain_map_to_image,
):
    return ComposableNode(
        namespace=camera_name,
        name=node_name,
        package='realsense_utils',
        plugin='realsense_utils::InfraGainCorrectionNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA',
            'gain_map_path': infra_gain_map_path,
            'resize_gain_map_to_image': resize_gain_map_to_image,
        }],
        remappings=[
            ('input/image', input_topic),
            ('~/output/image', output_topic),
        ],
        extra_arguments=[{'use_intra_process_comms': intra_process_comms}],
    )


def _delayed_param_set_action(camera_name: str, param_name: str, value: str, period: float):
    # RealSense dynamic sensor params are declared after the device finishes starting.
    # Wait for the param to exist; if this camera never exposes it, skip quietly.
    script = (
        'deadline=$((SECONDS + 20)); '
        'while ! ros2 param describe "$1" "$2" >/dev/null 2>&1; do '
        '  if [ "$SECONDS" -ge "$deadline" ]; then '
        '    echo "Skipping emitter retoggle for $1: $2 was not declared"; '
        '    exit 0; '
        '  fi; '
        '  sleep 0.5; '
        'done; '
        'ros2 param set "$1" "$2" "$3"'
    )
    return TimerAction(
        period=period,
        actions=[ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                script,
                '--',
                f'/{camera_name}',
                param_name,
                value,
            ],
            output='screen',
        )],
    )


def _add_cameras(context):
    container_name = LaunchConfiguration('container_name').perform(context)
    camera_serial_numbers_arg = LaunchConfiguration('camera_serial_numbers').perform(context)
    intra_process_comms = LaunchConfiguration('intra_process_comms')
    enable_infra_gain_correction = (
        LaunchConfiguration('enable_infra_gain_correction').perform(context).lower() == 'true'
    )
    infra_gain_map_path = LaunchConfiguration('infra_gain_map_path')
    resize_gain_map_to_image = LaunchConfiguration('resize_gain_map_to_image')
    infra_enable_auto_exposure = (
        LaunchConfiguration('infra_enable_auto_exposure').perform(context).lower() == 'true'
    )
    infra_exposure = LaunchConfiguration('infra_exposure').perform(context)
    infra_gain = LaunchConfiguration('infra_gain').perform(context)
    num_cameras = int(LaunchConfiguration('num_cameras').perform(context))
    run_standalone = LaunchConfiguration('run_standalone').perform(context).lower() == 'true'
    auto_retoggle_emitter = (
        LaunchConfiguration('auto_retoggle_emitter_on_off').perform(context).lower() == 'true'
    )
    emitter_retoggle_delay_sec = float(
        LaunchConfiguration('emitter_retoggle_delay_sec').perform(context)
    )
    package_share_dir = get_package_share_directory('realsense_utils')
    flashing_config = _load_camera_config(
        os.path.join(package_share_dir, 'config', 'sensors', 'realsense_emitter_flashing.yaml')
    )
    emitter_on_config = _load_camera_config(
        os.path.join(package_share_dir, 'config', 'sensors', 'realsense_emitter_on.yaml')
    )

    if camera_serial_numbers_arg == '':
        camera_serial_numbers = [None]
    else:
        camera_serial_numbers = camera_serial_numbers_arg.split(',')

    if num_cameras > len(camera_serial_numbers):
        raise RuntimeError(
            f'num_cameras={num_cameras} exceeds the provided serial numbers ({len(camera_serial_numbers)}).'
        )

    run_splitter_list = _default_run_splitter_list(len(camera_serial_numbers))

    actions = []
    for idx in range(num_cameras):
        camera_name = f'camera{idx}'
        serial_number = camera_serial_numbers[idx]
        run_splitter = run_splitter_list[idx]
        config_params = flashing_config if run_splitter else emitter_on_config

        nodes = [
            _camera_node(
                camera_name=camera_name,
                config_params=config_params,
                intra_process_comms=intra_process_comms,
                serial_number=serial_number,
            )
        ]
        if run_splitter:
            nodes.append(_splitter_node(camera_name, intra_process_comms, enable_infra_gain_correction))
            if enable_infra_gain_correction:
                nodes.extend([
                    _infra_gain_correction_node(
                        camera_name=camera_name,
                        node_name='infra_1_gain_correction_node',
                        intra_process_comms=intra_process_comms,
                        input_topic=f'/{camera_name}/realsense_splitter_node/raw/infra_1',
                        output_topic=f'/{camera_name}/realsense_splitter_node/output/infra_1',
                        infra_gain_map_path=infra_gain_map_path,
                        resize_gain_map_to_image=resize_gain_map_to_image,
                    ),
                    _infra_gain_correction_node(
                        camera_name=camera_name,
                        node_name='infra_2_gain_correction_node',
                        intra_process_comms=intra_process_comms,
                        input_topic=f'/{camera_name}/realsense_splitter_node/raw/infra_2',
                        output_topic=f'/{camera_name}/realsense_splitter_node/output/infra_2',
                        infra_gain_map_path=infra_gain_map_path,
                        resize_gain_map_to_image=resize_gain_map_to_image,
                    ),
                ])

        actions.append(
            TimerAction(
                period=idx * 10.0,
                actions=[LoadComposableNodes(
                    target_container=container_name,
                    composable_node_descriptions=nodes,
                )],
            )
        )
        if run_splitter and auto_retoggle_emitter:
            actions.append(
                _delayed_param_set_action(
                    camera_name=camera_name,
                    param_name='depth_module.emitter_on_off',
                    value='false',
                    period=idx * 10.0 + emitter_retoggle_delay_sec,
                )
            )
            actions.append(
                _delayed_param_set_action(
                    camera_name=camera_name,
                    param_name='depth_module.enable_auto_exposure',
                    value='true' if infra_enable_auto_exposure else 'false',
                    period=idx * 10.0 + emitter_retoggle_delay_sec + 1.0,
                )
            )
            if not infra_enable_auto_exposure and infra_exposure:
                actions.append(
                    _delayed_param_set_action(
                        camera_name=camera_name,
                        param_name='depth_module.exposure',
                        value=infra_exposure,
                        period=idx * 10.0 + emitter_retoggle_delay_sec + 2.0,
                    )
                )
            if not infra_enable_auto_exposure and infra_gain:
                actions.append(
                    _delayed_param_set_action(
                        camera_name=camera_name,
                        param_name='depth_module.gain',
                        value=infra_gain,
                        period=idx * 10.0 + emitter_retoggle_delay_sec + 3.0,
                    )
                )
            actions.append(
                _delayed_param_set_action(
                    camera_name=camera_name,
                    param_name='depth_module.emitter_on_off',
                    value='true',
                    period=idx * 10.0 + emitter_retoggle_delay_sec + 4.0,
                )
            )
    return actions


def generate_launch_description():
    container_name = LaunchConfiguration('container_name')
    run_standalone = LaunchConfiguration('run_standalone')

    return LaunchDescription([
        DeclareLaunchArgument('container_name', default_value='nvblox_container'),
        DeclareLaunchArgument('run_standalone', default_value='False'),
        DeclareLaunchArgument('camera_serial_numbers', default_value=''),
        DeclareLaunchArgument('num_cameras', default_value='1'),
        DeclareLaunchArgument('intra_process_comms', default_value='True'),
        DeclareLaunchArgument('auto_retoggle_emitter_on_off', default_value='False'),
        DeclareLaunchArgument('emitter_retoggle_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('enable_infra_gain_correction', default_value='True'),
        DeclareLaunchArgument('infra_enable_auto_exposure', default_value='true'),
        DeclareLaunchArgument('infra_exposure', default_value=''),
        DeclareLaunchArgument('infra_gain', default_value=''),
        DeclareLaunchArgument(
            'infra_gain_map_path',
            default_value='/tmp/radial_vignette_map_20260411_201754.npy',
        ),
        DeclareLaunchArgument('resize_gain_map_to_image', default_value='True'),
        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            condition=IfCondition(run_standalone),
        ),
        OpaqueFunction(function=_add_cameras),
    ])
