import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle dynamic parameters"""
    
    yanzj_ros2_bridge_path = get_package_share_directory('yanzj_ros2_bridge')
    
    # Get the use_simple_urdf parameter value
    use_simple_urdf = LaunchConfiguration('use_simple_urdf')
    simple_urdf = use_simple_urdf.perform(context)
    print(f"use_simple_urdf parameter value: {simple_urdf}")
    
    # Get the use_sim_time parameter value for consistent time synchronization
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = use_sim_time.perform(context)
    print(f"use_sim_time parameter value: {sim_time}")

    ranger_launch = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ranger_base'), 
                    'launch', 
                    "ranger_mini_v3_driver.launch.py"
                )
            ])
        ),  
    ])

    xarm_launch = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    # get_package_share_directory('xarm_api'), 
                    yanzj_ros2_bridge_path,
                    'launch', 
                    "xarm6_driver.launch.py"
                )
            ]),
            launch_arguments={
                'robot_ip': '192.168.1.233',
                'joint_states_rate': '50',  # 设置机械臂关节状态发布频率为50Hz
            }.items(),
        ),  
    ])


    camera_launch = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                yanzj_ros2_bridge_path, 
                '/launch/d435i_high_resolution.launch.py'])),  
    ])

    lidar_launch = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                # get_package_share_directory('livox_ros_driver2'), 
                # '/launch_ROS2/msg_MID360_launch.py'])),  
                yanzj_ros2_bridge_path, 
                '/launch/livox_mid360_driver.launch.py'])),  
    ])

    # livox_repub_node = Node(  # 暂时不使用livox_repub_node，因为fast_lio2的输出可以直接输入到rtabmap中
    #     package='livox_repub_ros2',
    #     executable='livox_repub_node',
    #     name='livox_repub_node',
    #     output='screen'
    # )

    # fast_lio2_launch = LaunchDescription([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             # get_package_share_directory('livox_ros_driver2'), 
    #             # '/launch_ROS2/msg_MID360_launch.py'])),  
    #             yanzj_ros2_bridge_path, 
    #             '/launch/fast_lio2.launch.py'])),  
    # ])

    if simple_urdf == 'true':
        urdf_file_path = os.path.join(yanzj_ros2_bridge_path, 'urdf', "rangerminiv3_with_xarm6_simple.urdf")
    else:
        urdf_file_path = os.path.join(yanzj_ros2_bridge_path, 'urdf', "rangerminiv3_with_xarm6.urdf")
    
    # Read the combined URDF file
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # Robot state publisher for the combined robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='combined_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
        }]
    )

    # Custom joint state aggregator that handles time synchronization
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='combined_joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/ranger/joint_states', '/xarm/joint_states'],
            'use_sim_time': use_sim_time,
            'rate': 30.0,
        }]
    )


    # Static transform from camera to end effector
    static_tf_camera_to_eef = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera_to_eef",
        # arguments=['-0.09566', '0.0325', '0.02532', '-1.5707963', '-1.5707963', '3.1415926', 'link_eef', 'camera_link'],
        arguments=['0.10014', '-0.0175', '0.02489', '0', '-1.04719755', '3.1415926', 'link_eef', 'camera_link'],
        output="screen"
    )

    # # Static transform from robot base to lidar
    # static_tf_base_to_lidar = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_base_to_lidar",
    #     arguments=['0.29011', '0.0', '0.075', '-1.5707963', '0', '0', 'base_link', 'lidar_frame'],
    #     output="screen"
    # )


    # Static transform from fast lio2 lidar frame to livox lidar frame
    static_tf_lidar_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_to_livox",
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'lidar_frame', 'livox_frame'],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )


    # Static transform from livox frame to robot base
    static_tf_livox_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_livox_to_base",
        # arguments=['-0.29011', '0.0', '-0.075', '0', '1.5707963', '0', 'lidar_frame', 'base_link'],
        arguments=['0.0', '-0.29011', '-0.075', '1.5707963', '0', '0', 'livox_frame', 'base_link'],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )


    static_tf_base_to_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_footprint",
        arguments=['0.0', '0', '-0.325', '0', '0', '0', 'base_link', 'base_footprint'],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Custom RViz for TF visualization only
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(yanzj_ros2_bridge_path, 'rviz', 'show_node_start.rviz')],
    )

    return [
        camera_launch,
        lidar_launch,
        # livox_repub_node,
        static_tf_camera_to_eef,
        static_tf_lidar_to_livox,
        static_tf_livox_to_base,
        static_tf_base_to_footprint,
        xarm_launch,
        ranger_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_simple_urdf',
            default_value='true',
            description='Whether to use simple URDF (true) or full URDF (false) for both xarm6 and ranger robots.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated clock for consistent time synchronization across all nodes.'
        ),
        OpaqueFunction(function=launch_setup),
    ])