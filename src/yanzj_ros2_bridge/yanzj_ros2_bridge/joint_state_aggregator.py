#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import time
from typing import Dict, List, Optional


class JointStateAggregator(Node):
    """
    Custom joint state aggregator that handles joint states from multiple robots
    and ensures data continuity even when one robot stops publishing.
    """
    
    def __init__(self):
        super().__init__('joint_state_aggregator')
        
        # Parameters
        self.declare_parameter('ranger_topic', '/ranger/joint_states')
        self.declare_parameter('xarm_topic', '/xarm/joint_states')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('timeout_threshold', 0.5)  # seconds
        
        # Get parameters
        self.ranger_topic = self.get_parameter('ranger_topic').value
        self.xarm_topic = self.get_parameter('xarm_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value
        
        # Store latest joint states
        self.ranger_joint_states: Optional[JointState] = None
        self.xarm_joint_states: Optional[JointState] = None
        
        # Timestamps for timeout checking
        self.ranger_last_update = 0.0
        self.xarm_last_update = 0.0
        
        # Thread lock for data access
        self.lock = threading.Lock()
        
        # Publishers and subscribers
        self.joint_states_pub = self.create_publisher(
            JointState, 
            self.output_topic, 
            10
        )
        
        self.ranger_sub = self.create_subscription(
            JointState,
            self.ranger_topic,
            self.ranger_callback,
            10
        )
        
        self.xarm_sub = self.create_subscription(
            JointState,
            self.xarm_topic,
            self.xarm_callback,
            10
        )
        
        # Timer for publishing aggregated joint states
        self.timer = self.create_timer(
            1.0 / self.publish_frequency, 
            self.publish_aggregated_states
        )
        
        self.get_logger().info(f'Joint State Aggregator initialized')
        self.get_logger().info(f'Subscribing to: {self.ranger_topic}, {self.xarm_topic}')
        self.get_logger().info(f'Publishing to: {self.output_topic}')
        self.get_logger().info(f'Publish frequency: {self.publish_frequency} Hz')
    
    def ranger_callback(self, msg: JointState):
        """Callback for ranger joint states"""
        with self.lock:
            self.ranger_joint_states = msg
            self.ranger_last_update = time.time()
    
    def xarm_callback(self, msg: JointState):
        """Callback for xarm joint states"""
        with self.lock:
            self.xarm_joint_states = msg
            self.xarm_last_update = time.time()
    
    def publish_aggregated_states(self):
        """Publish aggregated joint states"""
        current_time = time.time()
        
        with self.lock:
            # Check if data is fresh enough
            ranger_fresh = (current_time - self.ranger_last_update) < self.timeout_threshold
            xarm_fresh = (current_time - self.xarm_last_update) < self.timeout_threshold
            
            # Create aggregated joint state
            aggregated_msg = JointState()
            aggregated_msg.header = Header()
            aggregated_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Add ranger joints if available and fresh
            if self.ranger_joint_states and ranger_fresh:
                aggregated_msg.name.extend(self.ranger_joint_states.name)
                aggregated_msg.position.extend(self.ranger_joint_states.position)
                if self.ranger_joint_states.velocity:
                    aggregated_msg.velocity.extend(self.ranger_joint_states.velocity)
                if self.ranger_joint_states.effort:
                    aggregated_msg.effort.extend(self.ranger_joint_states.effort)
            
            # Add xarm joints if available and fresh
            if self.xarm_joint_states and xarm_fresh:
                aggregated_msg.name.extend(self.xarm_joint_states.name)
                aggregated_msg.position.extend(self.xarm_joint_states.position)
                if self.xarm_joint_states.velocity:
                    aggregated_msg.velocity.extend(self.xarm_joint_states.velocity)
                if self.xarm_joint_states.effort:
                    aggregated_msg.effort.extend(self.xarm_joint_states.effort)
            
            # Only publish if we have some data
            if aggregated_msg.name:
                self.joint_states_pub.publish(aggregated_msg)
                
                # Log if any robot is not publishing fresh data
                if not ranger_fresh and self.ranger_joint_states:
                    self.get_logger().warn_once('Ranger joint states are stale')
                if not xarm_fresh and self.xarm_joint_states:
                    self.get_logger().warn_once('XArm joint states are stale')


def main(args=None):
    rclpy.init(args=args)
    
    node = JointStateAggregator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
