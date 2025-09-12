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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context: LaunchContext, *args, **kwargs):
  
  frame_id = LaunchConfiguration('frame_id')
  
  imu_topic = LaunchConfiguration('imu_topic')
  imu_used =  imu_topic.perform(context) != ''
  
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
    'approx_sync': True,  # TODO set True when use RGBD
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
    'subscribe_rgbd': True,
    'subscribe_odom_info': True,
    'subscribe_scan_cloud': True,
    'map_frame_id': 'map',
    'odom_frame_id': 'odom',  # FAST-LIO's odometry frame
    'odom_sensor_sync': True, # This will adjust camera position based on difference between lidar and camera stamps.
    # RTAB-Map's internal parameters are strings:
    'RGBD/ProximityMaxGraphDepth': '0',
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'true',  # TODO, 默认为false
    'RGBD/ForceOdom3DoF': 'false',       # 默认: true - Force odometry pose to be 3DoF if Reg/Force3DoF=true.
    'RGBD/MaxDepth': '3.0',
    'RGBD/MinDepth': '0.2',

    # Grid 相关参数 - 3D地面和斜坡环境优化
    'Grid/3D': 'true',                    # 启用3D栅格地图
    'Grid/CellSize': '0.05',              # 栅格分辨率 (米)
    'Grid/NormalsSegmentation': 'true',   # 启用地面分割，区分地面和障碍物
    'Grid/NormalK': '20',                 # 法向邻域
    'Grid/MaxGroundAngle': '30',          # 地面法向与全局地面法向夹角阈值（度）

    'Grid/MaxGroundHeight': '0.15',        # 最大地面高度 (米) - 允许一定坡度
    'Grid/MinGroundHeight': '-0.15',       # 最小地面高度 (米) - 允许下坡
    
    'Grid/MaxObstacleHeight': '3',      # 最大障碍物高度 (米) - 适合小车通过
    'Grid/GroundIsObstacle': 'false',     # 地面不视为障碍物

    'Grid/PreVoxelFiltering': 'true',     # 预处理体素滤波
    'Grid/NoiseFilteringRadius': '0.12',   # 噪声过滤半径
    'Grid/NoiseFilteringMinNeighbors': '8', # 最小邻居数量

    'Grid/RangeMax': '3.0',               # 最大检测距离 (米)
    'Grid/RangeMin': '0.2',               # 最小检测距离 (米)

    'Grid/FootprintLength': '0.74',        # 机器人长度 (米) - 根据Ranger尺寸
    'Grid/FootprintWidth': '0.5',         # 机器人宽度 (米) - 根据Ranger尺寸
    'Grid/FootprintHeight': '1.4',        # 机器人高度 (米) - 根据Ranger-xarm尺寸

    'Grid/RayTracing': 'true',
    'Grid/Sensor': "1",                   # Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).
    
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',

    'Reg/Force3DoF': 'false',
    'Reg/RepeatOnce': 'true',
    'Reg/Strategy': '2',  # 0=Vis, 1=Icp, 2=VisIcp
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
  
  remappings.append(('rgb/image', rgb_image_topic))
  remappings.append(('rgb/camera_info', rgb_camera_info_topic))
  remappings.append(('depth/image', depth_image_topic))
  remappings.append(('grid_map', 'map'))
  
  nodes = [
    Node(
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
    
    # RTABMap 节点延迟启动，等待 FAST-LIO2 准备就绪
    TimerAction(
      period=5.0,  # 延迟 5 秒启动
      actions=[
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
    )
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
      'frame_id', default_value='base_footprint',
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

    