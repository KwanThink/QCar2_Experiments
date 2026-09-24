import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


# Create the OAMPC hardware, localization, map-processing, and controller launch description.
def generate_launch_description():
    bringup_dir = get_package_share_directory('qcar2_nodes')
    package_prefix = get_package_prefix('qcar2_nodes')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params_file = LaunchConfiguration('localization_params_file')
    oampc_params_file = LaunchConfiguration('oampc_params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    device_type = LaunchConfiguration('device_type')
    obstacle_map_file = '/home/nvidia/Maps/obstacle_map_oampc_Mk1c.yaml'
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_localization_params = RewrittenYaml(
        source_file=localization_params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        },
        convert_types=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace',
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the localization + OAMPC stack',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/nvidia/Maps/esynov_1obs.yaml',
        # default_value='/home/nvidia/Maps/esynov_4obs.yaml',
        description='Full path to the map yaml file',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true',
    )

    declare_localization_params_file_cmd = DeclareLaunchArgument(
        'localization_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_localization_oampc.yaml'),
        description='Full path to the localization parameter file',
    )

    declare_oampc_params_file_cmd = DeclareLaunchArgument(
        'oampc_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_oampc.yaml'),
        description='Full path to the QCar2 OAMPC parameter file',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically bring map_server and amcl to the active state',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level',
    )

    declare_device_type_cmd = DeclareLaunchArgument(
        'device_type',
        default_value='physical',
        description='physical or virtual',
    )

    qcar2_hardware_node = Node(
        package='qcar2_nodes',
        executable='qcar2_hardware',
        name='qcar2_hardware',
        output='screen',
        parameters=[
            {'device_type': device_type},
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    qcar2_odometry_node = Node(
        package='qcar2_nodes',
        executable='qcar2_odometry',
        name='qcar2_odometry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lidar_node = Node(
        package='qcar2_nodes',
        executable='lidar',
        name='lidar',
        output='screen',
        parameters=[
            {'device_type': device_type},
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    fixed_lidar_frame_physical = Node(
        package='qcar2_nodes',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_localization_params],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_localization_params],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
            {'bond_timeout': 10.0},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    generate_obstacle_map_process = ExecuteProcess(
        cmd=[
            os.path.join(package_prefix, 'lib', 'qcar2_nodes', 'generate_obstacle_map'),
            '--map', map_yaml_file,
            '--output', obstacle_map_file,
        ],
        output='screen',
    )

    qcar2_oampc_controller_node = Node(
        package='qcar2_nodes',
        executable='qcar2_oampc_controller',
        name='qcar2_oampc_controller',
        output='screen',
        parameters=[
            oampc_params_file,
            {'source_config_file': oampc_params_file},
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    hardware_localization_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        qcar2_hardware_node,
        qcar2_odometry_node,
        lidar_node,
        fixed_lidar_frame_physical,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization_node,
    ])

    controller_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        qcar2_oampc_controller_node,
    ])

    # Start the OAMPC controller only when obstacle-map generation succeeds.
    def start_controller_after_map_generation(event, context):
        del context
        if event.returncode == 0:
            return [controller_group]
        return [
            LogInfo(
                msg=(
                    'ERROR: generate_obstacle_map failed with return code '
                    f'{event.returncode}; qcar2_oampc_controller will not start.'
                )
            )
        ]

    obstacle_map_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_obstacle_map_process,
            on_exit=start_controller_after_map_generation,
        )
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_localization_params_file_cmd)
    ld.add_action(declare_oampc_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_device_type_cmd)

    ld.add_action(hardware_localization_group)
    ld.add_action(obstacle_map_exit_handler)
    ld.add_action(generate_obstacle_map_process)

    return ld
