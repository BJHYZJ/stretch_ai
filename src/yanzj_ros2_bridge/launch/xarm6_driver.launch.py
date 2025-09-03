#!/usr/bin/env python3
# Integrated xarm6 driver launch script
# Contains all necessary components for joint states and TF transforms without rviz

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.uf_robot_utils import generate_robot_api_params, get_xacro_content


def launch_setup(context, *args, **kwargs):
    # Launch Arguments
    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    dof = LaunchConfiguration('dof', default='6')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    use_simple_urdf = LaunchConfiguration('use_simple_urdf', default='true')
    
    # Robot API Parameters
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), 
        node_name='ufactory_driver'
    )
    
    # Robot Description (URDF)
    xacro_file_path = os.path.join(
        get_package_share_directory('xarm_description'), 
        'urdf', 
        'xarm_device.urdf.xacro'
    )
    robot_description = {
        'robot_description': get_xacro_content(
            context,
            xacro_file=xacro_file_path,
            dof=dof,
            robot_type='xarm',
            prefix='',
            hw_ns=hw_ns,
            limited=False,
            effort_control=False,
            velocity_control=False,
            model1300=False,
            robot_sn='',
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            kinematics_suffix='',
            ros2_control_plugin='uf_robot_hardware/UFRobotSystemHardware',
            add_gripper=False,
            add_vacuum_gripper=False,
            add_bio_gripper=False,
            add_realsense_d435i=False,
            add_d435i_links=True,
            add_other_geometry=False,
            geometry_type='box',
            geometry_mass=0.1,
            geometry_height=0.1,
            geometry_radius=0.1,
            geometry_length=0.1,
            geometry_width=0.1,
            geometry_mesh_filename='',
            geometry_mesh_origin_xyz='"0 0 0"',
            geometry_mesh_origin_rpy='"0 0 0"',
            geometry_mesh_tcp_xyz='"0 0 0"',
            geometry_mesh_tcp_rpy='"0 0 0"',
            use_simple_urdf=use_simple_urdf,
        )
    }
    
    # 1. Robot Driver Node - Hardware communication and joint states publishing
    robot_driver_node = Node(
        package='xarm_api',
        name='ufactory_driver',
        executable='xarm_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_params,
            {
                'robot_ip': robot_ip,
                'report_type': 'normal',
                'dof': dof,
                'add_gripper': False,
                'add_bio_gripper': False,
                'add_vacuum_gripper': False,
                'hw_ns': hw_ns.perform(context).strip('/'),
                'prefix': '',
                'baud_checkset': True,
                'default_gripper_baud': 2000000,
                'joint_states_rate': -1,
            },
        ]
    )
    
    # 2. Robot State Publisher - Publishes robot_description and TF transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='xarm_robot_state_publisher',  # 避免与ranger冲突
        output='screen',
        parameters=[robot_description],
    )
    
    # 3. Joint State Publisher - Aggregates joint state information
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='xarm_joint_state_publisher',  # 避免与ranger冲突
        output='screen',
        parameters=[{
            'source_list': ['{}/joint_states'.format(hw_ns.perform(context))]
        }],
    )
    
    return [
        robot_driver_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Robot IP address
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.233',
            description='IP address of the xarm6 robot.',
        ),
        # Hardware namespace
        DeclareLaunchArgument(
            'hw_ns',
            default_value='xarm',
            description='Hardware namespace for xarm_driver.',
        ),
        # Degrees of freedom
        DeclareLaunchArgument(
            'dof',
            default_value='6',
            description='Degrees of freedom of the manipulator.',
        ),
        # Attachment frame
        DeclareLaunchArgument(
            'attach_to',
            default_value='world',
            description='Parent frame to attach robot base to.',
        ),
        # Attachment position
        DeclareLaunchArgument(
            'attach_xyz',
            default_value='"0 0 0"',
            description='XYZ position of robot attachment (e.g., "1.0 0.5 0.8").',
        ),
        # Attachment orientation
        DeclareLaunchArgument(
            'attach_rpy',
            default_value='"0 0 0"',
            description='RPY orientation of robot attachment (e.g., "0 0 1.57").',
        ),
        # Use simple URDF
        DeclareLaunchArgument(
            'use_simple_urdf',
            default_value='true',
            description='Whether to use simple URDF (true) or full URDF (false).',
        ),
        
        OpaqueFunction(function=launch_setup),
    ])