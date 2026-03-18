#!/usr/bin/env python3
"""
check_topics.py - Diagnostic script to check ROS2 topics and TF tree

Run this while your simulation is running to diagnose issues.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time


class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        
        print('\n' + '=' * 70)
        print('ROS2 TOPIC CHECKER - Diagnostic Tool')
        print('=' * 70)
        
        # Get all topics
        self.check_all_topics()
        
        # Try to subscribe to odometry
        self.check_odometry_topic()
        
    def check_all_topics(self):
        """List all available topics."""
        print('\n[1] Checking all available topics...')
        print('-' * 70)
        
        topic_list = self.get_topic_names_and_types()
        
        if not topic_list:
            print('⚠ WARNING: No topics found!')
            return
        
        print(f'Found {len(topic_list)} topics:')
        
        # Filter for relevant topics
        odom_topics = []
        tf_topics = []
        scan_topics = []
        
        for topic_name, topic_types in topic_list:
            if 'odom' in topic_name.lower():
                odom_topics.append((topic_name, topic_types))
            elif 'tf' in topic_name.lower():
                tf_topics.append((topic_name, topic_types))
            elif 'scan' in topic_name.lower():
                scan_topics.append((topic_name, topic_types))
        
        # Print odometry topics
        if odom_topics:
            print(f'\n✓ Odometry-related topics ({len(odom_topics)}):')
            for name, types in odom_topics:
                print(f'  - {name}')
                for t in types:
                    print(f'      Type: {t}')
        else:
            print('\n✗ No odometry topics found!')
        
        # Print TF topics
        if tf_topics:
            print(f'\n✓ TF-related topics ({len(tf_topics)}):')
            for name, types in tf_topics:
                print(f'  - {name}')
        else:
            print('\n✗ No TF topics found!')
        
        # Print scan topics
        if scan_topics:
            print(f'\n✓ Scan-related topics ({len(scan_topics)}):')
            for name, types in scan_topics:
                print(f'  - {name}')
        
    def check_odometry_topic(self):
        """Try to subscribe to the odometry topic."""
        print('\n' + '-' * 70)
        print('[2] Testing odometry topic subscription...')
        print('-' * 70)
        
        self.odom_received = False
        self.odom_count = 0
        
        # Subscribe to the expected topic
        self.subscription = self.create_subscription(
            Odometry,
            '/model/ackermann_car/odometry',
            self.odom_callback,
            10)
        
        print('Subscribed to: /model/ackermann_car/odometry')
        print('Waiting for messages (timeout: 5 seconds)...')
        
        # Wait for messages
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_received:
                break
        
        if self.odom_received:
            print(f'✓ SUCCESS: Received {self.odom_count} odometry message(s)')
        else:
            print('✗ FAILED: No odometry messages received!')
            print('\nPossible issues:')
            print('  1. Gazebo simulation not running')
            print('  2. Bridge not working (ros_gz_bridge)')
            print('  3. Robot not spawned yet')
            print('  4. Topic name mismatch')
    
    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.odom_count += 1
        
        if not self.odom_received:
            self.odom_received = True
            print('\n' + '=' * 70)
            print('✓ FIRST ODOMETRY MESSAGE RECEIVED!')
            print('=' * 70)
            print(f'Frame ID: {msg.header.frame_id}')
            print(f'Child Frame ID: {msg.child_frame_id}')
            print(f'Position: x={msg.pose.pose.position.x:.3f}, '
                  f'y={msg.pose.pose.position.y:.3f}, '
                  f'z={msg.pose.pose.position.z:.3f}')
            print('=' * 70)


def main(args=None):
    """Main function."""
    
    print('\nInitializing ROS2...')
    rclpy.init(args=args)
    
    node = TopicChecker()
    
    print('\n' + '=' * 70)
    print('Diagnostic check complete!')
    print('=' * 70)
    print('\nNext steps:')
    print('1. If odometry topic is working, run odom_to_tf.py')
    print('2. If odometry topic is NOT working, check:')
    print('   - Is Gazebo running?')
    print('   - Is the robot spawned?')
    print('   - Is the bridge running?')
    print('=' * 70 + '\n')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
