#!/usr/bin/env python3
"""
Keyboard Override Node for A* Planner
Run this in a separate terminal to control replanning

Press 'r' - Force replan
Press 'c' - Cancel path
Press 'q' - Quit
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def get_key(settings):
    """Get keyboard input without blocking"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class PlannerKeyboardControl(Node):
    def __init__(self):
        super().__init__('planner_keyboard_control')
        
        # Parameters
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        
        robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value
        
        # Subscribers
        self.create_subscription(
            PoseStamped,
            robot_pose_topic,
            self.robot_pose_callback,
            10
        )
        
        self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.goal_topic,
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            10
        )
        
        # State
        self.robot_pose = None
        self.current_goal = None
        self.current_path = None
        
        # Keyboard monitoring
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.keyboard_loop)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('PLANNER KEYBOARD CONTROL')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Press:')
        self.get_logger().info('  r - Replan (republish current goal)')
        self.get_logger().info('  c - Cancel path (publish empty path)')
        self.get_logger().info('  q - Quit')
        self.get_logger().info('=' * 70)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def path_callback(self, msg):
        if len(msg.poses) > 0:
            self.current_path = msg
            # Extract goal from path (last waypoint)
            self.current_goal = msg.poses[-1]

    def keyboard_loop(self):
        """Check for keyboard input"""
        key = get_key(self.settings)
        
        if key == 'r' or key == 'R':
            self.handle_replan()
        elif key == 'c' or key == 'C':
            self.handle_cancel()
        elif key == 'q' or key == 'Q':
            self.get_logger().info('Quit command received')
            rclpy.shutdown()

    def handle_replan(self):
        """Force replanning by republishing goal"""
        if self.current_goal is None:
            self.get_logger().warn('⚠ No active goal to replan')
            return
        
        if self.robot_pose is None:
            self.get_logger().warn('⚠ No robot pose available')
            return
        
        self.get_logger().info('🔄 Replanning triggered - republishing goal...')
        
        # Create fresh goal message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.current_goal.header.frame_id
        goal_msg.pose = self.current_goal.pose
        
        # Republish goal (triggers A* to replan)
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info('✓ Goal republished - A* should replan')

    def handle_cancel(self):
        """Cancel current path"""
        self.get_logger().info('✗ Cancelling path...')
        
        # Publish empty path
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        
        self.path_pub.publish(empty_path)
        
        self.current_goal = None
        self.current_path = None
        
        self.get_logger().info('✓ Path cancelled')

    def __del__(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerKeyboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()