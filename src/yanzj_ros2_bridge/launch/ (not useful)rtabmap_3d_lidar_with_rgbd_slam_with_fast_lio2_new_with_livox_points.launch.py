# Copy from: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d.launch.py

import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def launch_setup(context: LaunchContext, *args, **kwargs):

  FRAME_BASE_LINK = 'base_link'
  FRAME_ODOM      = 'odom'
  FRAME_MAP       = 'map'
  FRAME_LIDAR     = 'lidar_frame'
  FRAME_LIVOX     = 'livox_frame'

  TOPIC_LIVOX_POINTS = '/livox/points'
  TOPIC_LIVOX_IMU    = '/livox/imu'
  TOPIC_ODOM_FASTLIO  = '/fast_lio2/Odometry'

  TOPIC_RGB   = '/camera/color/image_raw'
  TOPIC_DEPTH = '/camera/aligned_depth_to_color/image_raw'
  TOPIC_RGB_INFO = '/camera/color/camera_info'

  NS = 'rtabmap_ranger_xarm'

  USE_SIM_TIME = False
  VOXEL_SIZE_M = 0.1            # 下采样体素
  QOS_MODE     = 1              # 1 = reliable
  EXPECTED_LIDAR_RATE = 15.0    # Hz
  DESKEW_USE_SLERP = True

  shared_parameters = {
    'use_sim_time': USE_SIM_TIME,
    'frame_id': FRAME_BASE_LINK,
    'qos': QOS_MODE,
    'approx_sync': True,  # set True when use RGBD
    'wait_for_transform': 1.0,
    'tf_prefix': '',  # Use global TF frames
    # RTAB-Map's internal parameters are strings:
    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '10',
    'Icp/VoxelSize': str(VOXEL_SIZE_M),
    'Icp/Epsilon': '0.001',
    'Icp/PointToPlaneK': '20',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '3',
    'Icp/MaxCorrespondenceDistance': str(VOXEL_SIZE_M * 10.0),
    'Icp/Strategy': '1',
    'Icp/OutlierRatio': '0.7',
  }

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_rgbd': True,  # # for rgbd_sync
    'subscribe_odom_info': True,
    'subscribe_scan_cloud': True,
    'map_frame_id': FRAME_MAP,
    'odom_frame_id': FRAME_ODOM,  # FAST-LIO's odometry frame
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
    'Icp/CorrespondenceRatio': '0.2'
  }
  

  # 1) Livox 点云去畸变（固定帧 = livox_frame_stabilized）
  lidar_deskewing = Node(
      package='rtabmap_util', executable='lidar_deskewing', output='screen',
      namespace=NS,
      parameters=[{
          'use_sim_time': USE_SIM_TIME,
          'fixed_frame_id': FRAME_LIDAR,
          'wait_for_transform': 1.0,
          'slerp': DESKEW_USE_SLERP
      }],
      remappings=[('input_cloud', TOPIC_LIVOX_POINTS)]
  )

  # 2) IMU滤波（Madgwick）：/livox/imu -> /<NS>/imu/data
  imu_filter = Node(
      package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
      namespace=NS,
      parameters=[{
          'use_mag': False,
          'world_frame': 'enu',
          'publish_tf': False,
          'use_sim_time': USE_SIM_TIME
      }],
      remappings=[('imu/data_raw', TOPIC_LIVOX_IMU)]
  )


#   # 3) IMU 稳定帧：发布 TF（livox_frame -> livox_frame_stabilized）
#   imu_to_tf = Node(
#       package='rtabmap_util', executable='imu_to_tf', output='screen',
#       namespace=NS,
#       parameters=[{
#           'use_sim_time': USE_SIM_TIME,
#           'fixed_frame_id': FRAME_LIDAR,
#           'base_frame_id':  FRAME_LIVOX,
#           'wait_for_transform_duration': 1.0,
#           'use_sim_time': False
#       }],
#       # 订阅滤波后的 IMU：
#       remappings=[('imu', 'imu/data')]  # 在命名空间内 -> /rtabmap_ranger_xarm/imu/data
#   )


  # 4) RGBD 同步（相机）
  rgbd_sync = Node(
      package='rtabmap_sync', executable='rgbd_sync', output='screen',
      namespace=NS,
      parameters=[{'approx_sync': False, 'use_sim_time': USE_SIM_TIME}],
      remappings=[
          ('rgb/image',       TOPIC_RGB),
          ('rgb/camera_info', TOPIC_RGB_INFO),
          ('depth/image',     TOPIC_DEPTH),
          # 里程计、IMU 供下游用
          ('odom', TOPIC_ODOM_FASTLIO),
          ('imu',  'imu/data')
      ]
  )

  # 5) FAST-LIO2（里程计）
  fast_lio = Node(
      package='fast_lio',
      executable='fastlio_mapping',
      parameters=[
          PathJoinSubstitution([
              os.path.join(
                get_package_share_directory('yanzj_ros2_bridge'), 'config'
              ), 'fast_lio2_mid360.yaml'
          ]),
          {'use_sim_time': USE_SIM_TIME}
      ],
      output='screen'
  )


  # 6) RTAB-Map SLAM
  rtabmap = Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      namespace=NS,
      parameters=[shared_parameters, rtabmap_parameters, {'rgbd_cameras': 1}],
      remappings=[
          ('odom', TOPIC_ODOM_FASTLIO),
          ('imu',  'imu/data'),
          ('rgb/image',       TOPIC_RGB),
          ('rgb/camera_info', TOPIC_RGB_INFO),
          ('depth/image',     TOPIC_DEPTH),
          # 使用去畸变后的点云
          ('scan_cloud', TOPIC_LIVOX_POINTS + '/deskewed')
      ],
      arguments=['-d']  # 每次启动清空 ~/.ros/rtabmap.db
  )

  # 7) RTAB-Map 可视化（同样吃去畸变点云）
  rtabmap_viz = Node(
      package='rtabmap_viz', executable='rtabmap_viz', output='screen',
      namespace=NS,
      parameters=[shared_parameters, rtabmap_parameters],
      remappings=[
          ('odom', TOPIC_ODOM_FASTLIO),
          ('imu',  'imu/data'),
          ('rgb/image',       TOPIC_RGB),
          ('rgb/camera_info', TOPIC_RGB_INFO),
          ('depth/image',     TOPIC_DEPTH),
          ('scan_cloud', TOPIC_LIVOX_POINTS + '/deskewed')
      ]
  )

  nodes = [
    fast_lio,
    imu_filter,
    # imu_to_tf,
    lidar_deskewing,
    rgbd_sync,
    rtabmap,
    rtabmap_viz
  ]
      
  return nodes
  
def generate_launch_description():
  return LaunchDescription([
    OpaqueFunction(function=launch_setup),
  ])

    