import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    package_share = get_package_share_directory('demo2_apriltag_docking')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    turtlebot_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    turtlebot_navigation_share = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    target_tag_id = LaunchConfiguration('target_tag_id')
    guard_required = LaunchConfiguration('guard_required')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    world = os.path.join(package_share, 'worlds', 'docking_demo.sdf')
    map_yaml = os.path.join(package_share, 'maps', 'demo_map.yaml')
    nav2_params = os.path.join(package_share, 'config', 'turtlebot3_waffle_pi_nav2.yaml')
    docking_params = os.path.join(package_share, 'config', 'nav2_docking.yaml')
    dock_database = os.path.join(package_share, 'config', 'dock_database.yaml')
    dock_mapping = os.path.join(package_share, 'config', 'docks.yaml')
    apriltag_params = os.path.join(package_share, 'config', 'apriltag.yaml')
    robot_model = os.path.join(
        turtlebot_gazebo_share,
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf',
    )
    robot_bridge_params = os.path.join(
        turtlebot_gazebo_share,
        'params',
        'turtlebot3_waffle_pi_bridge.yaml',
    )

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={'dock_database': dock_database},
        convert_types=True,
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 "', world, '"'],
            'on_exit_shutdown': 'true',
        }.items(),
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2'}.items(),
        condition=UnlessCondition(headless),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_gazebo_share,
                'launch',
                'robot_state_publisher.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'waffle_pi',
            '-file',
            robot_model,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.01',
        ],
        output='screen',
    )
    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={robot_bridge_params}',
        ],
        output='screen',
    )
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true',
        }.items(),
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(
                turtlebot_navigation_share,
                'rviz',
                'tb3_navigation2.rviz',
            ),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
        output='screen',
    )
    tag_bridge = Node(
        package='demo2_apriltag_docking',
        executable='tag_pose_bridge',
        name='tag_pose_bridge',
        parameters=[
            docking_params,
            {'dock_mapping_file': dock_mapping, 'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
    task_bridge = Node(
        package='demo2_apriltag_docking',
        executable='docking_task_bridge',
        name='docking_task_bridge',
        parameters=[
            docking_params,
            {
                'dock_mapping_file': dock_mapping,
                'target_tag_id': ParameterValue(target_tag_id, value_type=int),
                'guard_required': ParameterValue(guard_required, value_type=bool),
                'use_sim_time': use_sim_time,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_tag_id', default_value='0'),
        DeclareLaunchArgument('guard_required', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(package_share, 'models'),
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(turtlebot_gazebo_share, 'models'),
        ),
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        robot_bridge,
        image_bridge,
        nav2,
        rviz_node,
        apriltag,
        tag_bridge,
        task_bridge,
    ])
