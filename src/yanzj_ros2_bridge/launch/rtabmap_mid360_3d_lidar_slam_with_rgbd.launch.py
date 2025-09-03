# Description:
#   Fusion 3D LiDAR + RGB-D SLAM using MID-360 LiDAR and D435i camera
#   Based on rtabmap demos with simplified configuration
#
# Example:
#   Launch your sensors first:
#   $ ros2 launch yanzj_ros2_bridge yanzj_node_start.launch.py
#
#   Then launch this fusion SLAM system:
#   $ ros2 launch yanzj_ros2_bridge rtabmap_mid360_3d_lidar_slam_with_rgbd.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    use_camera = LaunchConfiguration('use_camera')

    # ICP odometry parameters
    icp_odom_parameters = {
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': 'base_link',  # 使用base_link作为参考坐标系
        'OdomF2M/ScanSubtractRadius': '0.1',  # match voxel size
        'OdomF2M/ScanMaxSize': '15000'
    }

    # RTAB-Map parameters
    rtabmap_parameters = {
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': use_camera,
        'subscribe_scan_cloud': True,
        'use_action_for_goal': True,
        'odom_sensor_sync': True,  # 关闭严格时间同步
        # RTAB-Map's parameters should be strings:
        'Mem/NotLinkedNodesKept': 'false',
        'Grid/RangeMin': '0.5',  # ignore laser scan points on the robot itself
        'Grid/NormalsSegmentation': 'false',  # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight': '0.05',  # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight': '1',  # All points over 1 meter are ignored
        'Grid/RayTracing': 'true',  # Fill empty space
        'Grid/3D': 'false',  # Use 2D occupancy
        'RGBD/OptimizeMaxError': '0.3',  # Be more strict in accepting loop closures
    }

    # Shared parameters between different nodes
    shared_parameters = {
        'frame_id': 'base_link',  # 使用base_link作为参考坐标系，更稳定
        'use_sim_time': use_sim_time,
        # RTAB-Map's parameters should be strings:
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',  # we are moving on a 2D flat floor
        'Mem/NotLinkedNodesKept': 'false',
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '1.0',  # roughly 10x voxel size
        'Icp/PointToPlaneGroundNormalsUp': '0.9',
        'Icp/RangeMin': '0.5',
        'Icp/MaxTranslation': '3'
    }

    # Remappings for your specific topics
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('odom', 'icp_odom'),
        ('scan_cloud', '/livox/lidar'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', choices=['true', 'false'],
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),

        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Use camera for global loop closure / re-localization.'),

        # RGB-D synchronization node
        Node(
            condition=IfCondition(use_camera),
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'use_sim_time': use_sim_time, 'queue_size': 10}],  # 改为宽松同步
            remappings=remappings),

        # ICP odometry node
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[icp_odom_parameters, shared_parameters],
            remappings=remappings,
            arguments=["--ros-args", "--log-level", 'warn']),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_parameters, shared_parameters,
                {'Mem/IncrementalMemory': 'False',
                 'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        # Visualization node
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_parameters, shared_parameters],
            remappings=remappings),
    ])

    