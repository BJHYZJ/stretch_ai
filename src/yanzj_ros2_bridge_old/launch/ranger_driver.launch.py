#!/usr/bin/env python3
# Integrated ranger driver launch script
# Contains all necessary components for joint states and TF transforms without rviz

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle dynamic parameters"""
    
    # Launch Arguments
    port_name = LaunchConfiguration("port_name")
    robot_model = LaunchConfiguration("robot_model")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    update_rate = LaunchConfiguration("update_rate")
    odom_topic_name = LaunchConfiguration("odom_topic_name")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    use_simple_urdf = LaunchConfiguration("use_simple_urdf")

    # Get the actual values of parameters
    port_name_value = port_name.perform(context)
    robot_model_value = robot_model.perform(context)
    odom_frame_value = odom_frame.perform(context)
    base_frame_value = base_frame.perform(context)
    update_rate_value = update_rate.perform(context)
    odom_topic_name_value = odom_topic_name.perform(context)
    publish_odom_tf_value = publish_odom_tf.perform(context)
    use_simple_urdf_value = use_simple_urdf.perform(context)

    print(f"ranger_driver parameters:")
    print(f"  port_name: {port_name_value}")
    print(f"  robot_model: {robot_model_value}")
    print(f"  odom_frame: {odom_frame_value}")
    print(f"  base_frame: {base_frame_value}")
    print(f"  update_rate: {update_rate_value}")
    print(f"  odom_topic_name: {odom_topic_name_value}")
    print(f"  publish_odom_tf: {publish_odom_tf_value}")
    print(f"  use_simple_urdf: {use_simple_urdf_value}")

    # Ranger base node
    ranger_base_node = Node(
        package="ranger_base",
        executable="ranger_base_node",
        name="ranger_base_node",
        output="screen",
        parameters=[{
            "port_name": port_name_value,
            "robot_model": robot_model_value,
            "odom_frame": odom_frame_value,
            "base_frame": base_frame_value,
            "update_rate": int(update_rate_value),
            "odom_topic_name": odom_topic_name_value,
            "publish_odom_tf": publish_odom_tf_value == 'true',
        }],
    )

    # Robot description (URDF) - choose between simple and full
    ranger_description_path = get_package_share_path('ranger_description')
    
    # Choose between simple and full URDF based on parameter
    if use_simple_urdf_value == 'true':
        urdf_filename = 'ranger_mini3_v3_simple.urdf'
        print(f"Using simple URDF: {urdf_filename}")
    else:
        urdf_filename = 'ranger_mini3_v3.urdf'
        print(f"Using full URDF: {urdf_filename}")
    
    urdf_file_path = str(ranger_description_path / 'urdf' / urdf_filename)
    
    # Read URDF file content
    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    # Robot state publisher (for ranger only)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ranger_robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content,
                     'publish_frequency': 15.0}]
    )

    # Custom joint state publisher for ranger
    joint_state_publisher_node = Node(
        package='ranger_description',
        executable='ranger_joint_state_publisher',
        name='ranger_joint_state_publisher',
        output='screen',
    )

    nodes_to_return = [
        ranger_base_node, 
        robot_state_publisher_node, 
        joint_state_publisher_node
    ]
    
    return nodes_to_return


def generate_launch_description():
    return LaunchDescription([
        # CAN port name
        DeclareLaunchArgument(
            "port_name", 
            default_value="can0",
            description="CAN port name for ranger robot communication."
        ),
        # Robot model
        DeclareLaunchArgument(
            "robot_model", 
            default_value="ranger",
            description="Robot model name."
        ),
        # Odometry frame
        DeclareLaunchArgument(
            "odom_frame", 
            default_value="odom",
            description="Odometry frame name."
        ),
        # Base frame
        DeclareLaunchArgument(
            "base_frame", 
            default_value="base_link",
            description="Base frame name."
        ),
        # Update rate
        DeclareLaunchArgument(
            "update_rate", 
            default_value="50",
            description="Update rate for odometry publishing (Hz)."
        ),
        # Odometry topic name
        DeclareLaunchArgument(
            "odom_topic_name", 
            default_value="odom",
            description="Odometry topic name."
        ),
        # Publish odometry TF
        DeclareLaunchArgument(
            "publish_odom_tf", 
            default_value="true",
            description="Whether to publish odometry TF transform."
        ),
        # Use simple URDF
        DeclareLaunchArgument(
            "use_simple_urdf", 
            default_value="true",
            description="Whether to use simple URDF (true) or full URDF (false)."
        ),
        
        OpaqueFunction(function=launch_setup),
    ])
