#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JoyMorodimControl(Node):
    def __init__(self):
        super().__init__('joy_morodim_control')
        
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('joint_step', 0.05)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.joint_step = self.get_parameter('joint_step').value
        
        self.joint_positions = [0.0, 0.0, 0.0]
        
        self.update_rate = 0.05
        self.joint_velocities = [0.0, 0.0, 0.0]
        self.update_timer = self.create_timer(self.update_rate, self.update_joints)
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, 
                                             '/joint_trajectory_position_controller/follow_joint_trajectory')
        
        self.get_logger().info('Waiting for trajectory controller...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Trajectory controller connected!')
        
        self.get_logger().info('Joy Morodim Control Node started')
        self.get_logger().info('Controls:')
        self.get_logger().info('- Left stick: Base movement (forward/backward, rotation)')
        self.get_logger().info('- L2/R2: Joint 1 control')
        self.get_logger().info('- Triangle/X: Joint 2 control')
        self.get_logger().info('- L1/R1: Joint 3 control')
        
    def joy_callback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.axes[1] * self.max_linear_speed
        cmd_vel.angular.z = msg.axes[0] * self.max_angular_speed
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Joint 1 - L2/R2
        if msg.axes[2] < 0:  # L2 pressed
            self.joint_velocities[0] = self.joint_step / self.update_rate
        elif msg.axes[5] < 0:  # R2 pressed
            self.joint_velocities[0] = -self.joint_step / self.update_rate
        else:
            self.joint_velocities[0] = 0.0
            
        # Joint 2 - Triangle/X
        if msg.buttons[2] == 1:  # Triangle
            self.joint_velocities[1] = self.joint_step / self.update_rate
        elif msg.buttons[0] == 1:  # X
            self.joint_velocities[1] = -self.joint_step / self.update_rate
        else:
            self.joint_velocities[1] = 0.0
            
        # Joint 3 - L1/R1
        if msg.buttons[4] == 1:  # L1
            self.joint_velocities[2] = -self.joint_step / self.update_rate
        elif msg.buttons[5] == 1:  # R1
            self.joint_velocities[2] = self.joint_step / self.update_rate
        else:
            self.joint_velocities[2] = 0.0
    
    def update_joints(self):
        if any(abs(v) > 0.0001 for v in self.joint_velocities):
            for i in range(3):
                self.joint_positions[i] += self.joint_velocities[i] * self.update_rate
                
                if self.joint_positions[i] > 3.14:
                    self.joint_positions[i] = 3.14
                    self.joint_velocities[i] = 0.0
                elif self.joint_positions[i] < -3.14:
                    self.joint_positions[i] = -3.14
                    self.joint_velocities[i] = 0.0
            
            self.send_joint_trajectory()
    
    def send_joint_trajectory(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        goal_msg.trajectory.points = [point]
        
        self.trajectory_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMorodimControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()