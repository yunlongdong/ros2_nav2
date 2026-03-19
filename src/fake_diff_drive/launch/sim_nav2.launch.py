import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_pkg = get_package_share_directory('nav2_bringup_custom')
    fake_pkg = get_package_share_directory('fake_diff_drive')

    default_map = os.path.join(nav2_pkg, 'map', 'my_map.yaml')
    default_params = os.path.join(nav2_pkg, 'params', 'nav2_params.yaml')
    rviz_config = os.path.join(fake_pkg, 'rviz', 'nav2_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map)
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    stdout_linebuf = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Rewrite params with use_sim_time substitution
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # === Fake robot nodes ===
    fake_diff_drive = Node(
        package='fake_diff_drive',
        executable='fake_diff_drive_node',
        name='fake_diff_drive',
        output='screen')

    fake_scan = Node(
        package='fake_diff_drive',
        executable='fake_scan_node',
        name='fake_scan',
        output='screen')

    # === Map server ===
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'yaml_filename': map_yaml}],
        remappings=remappings)

    # === Nav2 navigation nodes ===
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                  ('cmd_vel_smoothed', 'cmd_vel')])

    # === Lifecycle manager for ALL nav2 nodes ===
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'map_server',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }])

    # === RViz2 ===
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        declare_autostart,
        stdout_linebuf,
        # Fake robot
        fake_diff_drive,
        fake_scan,
        # Nav2 stack
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        # Visualization
        rviz2,
    ])
