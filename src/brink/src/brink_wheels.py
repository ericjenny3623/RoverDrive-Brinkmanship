#!/usr/bin/env python
#
# inputs - gps, wheel_odom, imu
# outputs - motor_commands (init only), odometry
#
#
# init - drive back then forward and calculate yaw.
#

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64
import collections
import time


def wrapTo2Pi(a):
  return a % (np.pi*2.0)


class WheelMonitorNode:
    def __init__(self):
        # Inputs
        self.wheel_odom_sub = rospy.Subscriber(
            '/pitranger/out/wheel_odom', Odometry, self.wheel_odom_callback)
        self.wheel_cmd_sub = rospy.Subscriber(
            '/teleop/out/twist_cmd', Twist, self.wheel_cmd_callback)

        self.robot_orientation_sub_front = rospy.Subscriber(
            '/brink/out/robot_orientation/angle/front', Float64, self.front_angle_callback)
        self.robot_orientation_sub_right = rospy.Subscriber(
            '/brink/out/robot_orientation/angle/right', Float64, self.right_angle_callback)

        # Outputs
        # self.odom_pub = rospy.Publisher('/whereami/odom', Odometry, queue_size=10)

        # More inputs
        self.imu_sub = rospy.Subscriber('/xsens/data', Imu, self.imu_callback)

        self.cmdVels = np.zeros(2)
        self.wheelVels = np.zeros(2)
        self.frontAngle = 0.0
        self.rightangle = 0.0

    def wheel_odom_callback(self, msg):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        self.wheel_odom_fifo.append([lin, ang])

    def wheel_cmd_callback(self, msg):
        pass

    def front_angle_callback(self, msg):
        pass

    def right_angle_callback(self, msg):
        pass

    def imu_callback(self, msg):
        pass

    def broadcast_transform(self):
        self.br.sendTransform((self.pos[0], self.pos[1], 0),
                            quaternion_from_euler(0, 0, self.yaw),
                            rospy.Time.now(),
                            "base_link",
                            "map")


if __name__ == "__main__":
  rospy.init_node('wheelmonitor')

  node = WheelMonitorNode()

  rospy.spin()
