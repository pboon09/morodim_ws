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
        self.declare_parameter('joint_step', 0.1)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.joint_step = self.get_parameter('joint_step').value
        
        self.joint_positions = [0.0, 0.0, 0.0]  # joint_1, joint_2, joint_3
        
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,10)
        
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        
        self.trajectory_client = ActionClient(self,FollowJointTrajectory,'/joint_trajectory_position_controller/follow_joint_trajectory')
        
        self.get_logger().info('Waiting for trajectory controller...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Trajectory controller connected!')
        
        self.get_logger().info('Joy Morodim Control Node started')
        self.get_logger().info('Controls:')
        self.get_logger().info('- Left stick: Base movement (forward/backward, rotation)')
        self.get_logger().info('- D-pad: Joint 1 control (up/down)')
        self.get_logger().info('- Buttons: Joint 2 control (Y/A or Triangle/X)')
        self.get_logger().info('- Bumpers: Joint 3 control (LB/RB)')
        
    def joy_callback(self, msg):
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.axes[1] * self.max_linear_speed
        cmd_vel.angular.z = msg.axes[0] * self.max_angular_speed
        self.cmd_vel_pub.publish(cmd_vel)
        
        joint_changed = False
        
        # Joint 1 D-pad
        if msg.axes[7] > 0:  # D-pad up
            self.joint_positions[0] += self.joint_step
            joint_changed = True
        elif msg.axes[7] < 0:  # D-pad down
            self.joint_positions[0] -= self.joint_step
            joint_changed = True
        
        # Joint 2 Triangle / X
        if msg.buttons[2] == 1:  # Triangle
            self.joint_positions[1] += self.joint_step
            joint_changed = True
        elif msg.buttons[0] == 1:  # X
            self.joint_positions[1] -= self.joint_step
            joint_changed = True
        
        # Joint 3 LB / RB
        if msg.buttons[4] == 1:  # LB
            self.joint_positions[2] -= self.joint_step
            joint_changed = True
        elif msg.buttons[5] == 1:  # RB
            self.joint_positions[2] += self.joint_step
            joint_changed = True
        
        if joint_changed:
            self.send_joint_trajectory()
        
        self.prev_buttons = list(msg.buttons)
        self.prev_buttons.extend(msg.axes)
    
    def send_joint_trajectory(self):
        for i in range(len(self.joint_positions)):
            if self.joint_positions[i] > 3.14:
                self.joint_positions[i] = 3.14
            elif self.joint_positions[i] < -3.14:
                self.joint_positions[i] = -3.14
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.time_from_start = Duration(sec=0, nanosec=300000000)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Moving to: {self.joint_positions}')
        self.trajectory_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMorodimControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()