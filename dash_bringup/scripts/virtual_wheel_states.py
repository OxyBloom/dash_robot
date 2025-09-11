#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class VirtualWheelStates(Node):
    def __init__(self):
        super().__init__('virtual_wheel_states')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.287)
        self.declare_parameter('use_odom', True)   # if False use /cmd_vel
        self.declare_parameter('left_wheel_joint', 'lw_joint')
        self.declare_parameter('right_wheel_joint', 'rw_joint')
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.use_odom = self.get_parameter('use_odom').value
        self.l_name = self.get_parameter('left_wheel_joint').value
        self.r_name = self.get_parameter('right_wheel_joint').value

        self.last_t = time.time()
        self.theta_l = 0.0
        self.theta_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0

        if self.use_odom:
            self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        else:
            self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.update)

    def odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.update_wheel_rates(v, w)

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self.update_wheel_rates(v, w)

    def update_wheel_rates(self, v, w):
        self.w_l = (v - w * self.L / 2.0) / self.R
        self.w_r = (v + w * self.L / 2.0) / self.R

    def update(self):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        # Integrate
        self.theta_l += self.w_l * dt
        self.theta_r += self.w_r * dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.l_name, self.r_name]
        js.position = [self.theta_l, self.theta_r]
        js.velocity = [self.w_l, self.w_r]
        js.effort = []
        self.pub.publish(js)

def main():
    rclpy.init()
    rclpy.spin(VirtualWheelStates())
    rclpy.shutdown()

if __name__ == '__main__':
    main()