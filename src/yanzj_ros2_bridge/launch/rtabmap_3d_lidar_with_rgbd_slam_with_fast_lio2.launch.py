# Copy from: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d.launch.py

# Description:
#   In this example, we keep only minimal data to do LiDAR SLAM.
#
# Example:
#   Launch your lidar sensor:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   If an IMU is used, make sure TF between lidar/base frame and imu is
#     already calibrated. In this example, we assume the imu topic has 
#     already the orientation estimated, if not, you can use 
#     imu_filter_madgwick_node (with use_mag:=false publish_tf:=false)
#     and set imu_topic to output topic of the filter.
#
#   If a camera is used, make sure TF between lidar/base frame and camera is
#     already calibrated. To provide image data to this example, you should use
#     rtabmap_sync's rgbd_sync or stereo_sync node.
#
#   Launch the example by adjusting the lidar topic and base frame:
#   $ ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/velodyne_points frame_id:=velodyne

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  frame_id = LaunchConfiguration('frame_id')
  
  imu_topic = LaunchConfiguration('imu_topic')
  imu_used =  imu_topic.perform(context) != ''
  
  use_camera = LaunchConfiguration('use_camera')
  robot_ns = LaunchConfiguration('robot_ns')
  
  rgb_image_topic = LaunchConfiguration('rgb_image_topic')
  rgb_camera_info_topic = LaunchConfiguration('rgb_camera_info_topic')
  depth_image_topic = LaunchConfiguration('depth_image_topic')

  voxel_size = LaunchConfiguration('voxel_size')
  voxel_size_value = float(voxel_size.perform(context))
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Use FAST-LIO2's output point cloud which is already registered and in odom frame
  fast_lio_cloud_topic = LaunchConfiguration('fast_lio_cloud_topic')
  lidar_topic_deskewed = fast_lio_cloud_topic.perform(context)
  
  localization = LaunchConfiguration('localization').perform(context)
  localization = localization == 'true' or localization == 'True'
  
  # Rule of thumb:
  max_correspondence_distance = voxel_size_value * 10.0

  shared_parameters = {
    'use_sim_time': use_sim_time,
    'frame_id': frame_id,
    'qos': LaunchConfiguration('qos'),
    'approx_sync': use_camera,  # TODO set True when use RGBD
    'wait_for_transform': 0.2,
    # RTAB-Map's internal parameters are strings:
    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '10',
    'Icp/VoxelSize': str(voxel_size_value),
    'Icp/Epsilon': '0.001',
    'Icp/PointToPlaneK': '20',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '3',
    'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
    'Icp/Strategy': '1',
    'Icp/OutlierRatio': '0.7',
  }

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_rgbd': use_camera,
    'subscribe_odom_info': True,
    'subscribe_scan_cloud': True,
    'map_frame_id': 'new_map',
    'odom_frame_id': 'odom',  # FAST-LIO's odometry frame
    'odom_sensor_sync': True, # This will adjust camera position based on difference between lidar and camera stamps.
    # RTAB-Map's internal parameters are strings:
    'RGBD/ProximityMaxGraphDepth': '0',
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'false',  # TODO, 默认为false
    'RGBD/ForceOdom3DoF': 'true',        # 默认: true - 强制3自由度里程计。false=允许6DoF，更适应震动环境
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',
    'Reg/Strategy': '1',
    'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context))
  }
  
  arguments = []
  if localization:
    rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
    rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
  else:
    arguments.append('-d') # This will delete the previous database (~/.ros/rtabmap.db)
  
  # Use FAST-LIO odometry instead of ICP odometry
  remappings = [('odom', '/fast_lio2/Odometry')]
  if imu_used:
    remappings.append(('imu', 'imu/data'))  # notice namespace problem
  else:
    remappings.append(('imu', 'imu_not_used'))
  
  # Add camera remappings only when use_camera is true
  use_camera_value = use_camera.perform(context)
  if use_camera_value == 'true':
    # Assert that camera topics are properly configured
    rgb_topic_value = rgb_image_topic.perform(context)
    rgb_info_value = rgb_camera_info_topic.perform(context)
    depth_topic_value = depth_image_topic.perform(context)
    
    assert rgb_topic_value, "RGB image topic must be specified when use_camera=true"
    assert rgb_info_value, "RGB camera info topic must be specified when use_camera=true"
    assert depth_topic_value, "Depth image topic must be specified when use_camera=true"
    
    remappings.append(('rgb/image', rgb_image_topic))
    remappings.append(('rgb/camera_info', rgb_camera_info_topic))
    remappings.append(('depth/image', depth_image_topic))
  
  nodes = [
    Node(
      condition=IfCondition(use_camera),
      package='rtabmap_sync', executable='rgbd_sync', output='screen',
      namespace=robot_ns,
      parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
      remappings=remappings),
      
    # fast_lio2 node
    Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([os.path.join(
          get_package_share_directory('yanzj_ros2_bridge'), 'config'
        ), 'fast_lio2_mid360.yaml']), {'use_sim_time': use_sim_time}],
        output='screen'
    ),
    
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      namespace=robot_ns,
      parameters=[shared_parameters, rtabmap_parameters, 
                  {'rgbd_cameras': 1}],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
      arguments=arguments), 
  
    Node(
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      namespace=robot_ns,
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=remappings + [('scan_cloud', lidar_topic_deskewed)])  # Use FAST-LIO2 output directly
  ]

  print(imu_used)
  if imu_used:
    nodes.append(
      Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        namespace=robot_ns,
        parameters=[{'use_mag': False, 
                      'world_frame':'enu', 
                      'publish_tf':False}],
        remappings=[('imu/data_raw', imu_topic)]))

      
  return nodes
  
def generate_launch_description():
  return LaunchDescription([

    # Launch arguments
    DeclareLaunchArgument(
      'use_sim_time', default_value='false',
      description='Use simulated clock.'),
    
    DeclareLaunchArgument(
      'frame_id', default_value='base_link',
      description='Base frame of the robot.'),
    
    DeclareLaunchArgument(
      'localization', default_value='false',
      description='Localization mode.'),

    DeclareLaunchArgument(
      'fast_lio_cloud_topic', default_value='/fast_lio2/cloud_registered',
      description='FAST-LIO2 registered point cloud topic.'),

    DeclareLaunchArgument(
      'imu_topic', default_value='/livox/imu',
      description='IMU topic (ignored if empty).'),
    
    DeclareLaunchArgument(
      'use_camera', default_value='true',
      description='Use camera for global loop closure / re-localization.'),

    DeclareLaunchArgument(
      'robot_ns', default_value='rtabmap_ranger_xarm',
      description='Robot namespace.'),

    DeclareLaunchArgument(
      'rgb_image_topic', default_value='/camera/color/image_raw',
      description='RGB image topic.'),

    DeclareLaunchArgument(
      'rgb_camera_info_topic', default_value='/camera/color/camera_info',
      description='RGB camera info topic.'),

    DeclareLaunchArgument(
      'depth_image_topic', default_value='/camera/aligned_depth_to_color/image_raw',
      description='Depth image topic.'),

    DeclareLaunchArgument(
      'voxel_size', default_value='0.1',
      description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),
    
    DeclareLaunchArgument(
      'min_loop_closure_overlap', default_value='0.2',
      description='Minimum scan overlap pourcentage to accept a loop closure.'),

    DeclareLaunchArgument(
      'qos', default_value='1',
      description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

    OpaqueFunction(function=launch_setup),
  ])

    