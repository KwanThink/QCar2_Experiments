import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


# Create the FLMPC hardware, localization, and controller launch description.
def generate_launch_description():
    bringup_dir = get_package_share_directory('qcar2_nodes')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params_file = LaunchConfiguration('localization_params_file')
    flmpc_params_file = LaunchConfiguration('flmpc_params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    device_type = LaunchConfiguration('device_type')
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
        description='Whether to apply a namespace to the localization + FLMPC stack',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        # default_value='/home/nvidia/Maps/d215_map.yaml',
        default_value='/home/nvidia/Maps/esynov_map.yaml',
        description='Full path to the map yaml file',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true',
    )

    declare_localization_params_file_cmd = DeclareLaunchArgument(
        'localization_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_localization_flmpc.yaml'),
        description='Full path to the localization parameter file',
    )

    declare_flmpc_params_file_cmd = DeclareLaunchArgument(
        'flmpc_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_flmpc.yaml'),
        description='Full path to the QCar2 FLMPC parameter file',
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

    qcar2_flmpc_controller_node = Node(
        package='qcar2_nodes',
        executable='qcar2_flmpc_controller',
        name='qcar2_flmpc_controller',
        output='screen',
        parameters=[
            flmpc_params_file,
            {'source_config_file': flmpc_params_file},
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    flmpc_group = GroupAction([
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
        qcar2_flmpc_controller_node,
    ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_localization_params_file_cmd)
    ld.add_action(declare_flmpc_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_device_type_cmd)

    ld.add_action(flmpc_group)

    return ld
