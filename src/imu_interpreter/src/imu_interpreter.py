#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def callback(msg, args):
    qx = msg.orientation.x
    qy = msg.orientation.y
    qz = msg.orientation.z
    qw = msg.orientation.w
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    moving_average = args[0]
    n_samples = args[1]
    if (len(moving_average) < n_samples):
        moving_average.remove(moving_average[0])
    moving_average.append([roll, pitch, yaw])
    orientations = np.mean(np.array(moving_average), axis=0)
    roll_ave = orientations[0]
    pitch_ave = orientations[1]
    yaw_ave = orientations[2]

    print("Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
        roll * 180.0/np.pi, 
        pitch * 180.0/np.pi, 
        yaw * 180.0/np.pi))
    print("Filtered Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
        roll_ave * 180.0/np.pi, 
        pitch_ave * 180.0/np.pi, 
        yaw_ave * 180.0/np.pi))


if __name__=="__main__":
    rospy.init_node('imu_interpreter')
    moving_average = []
    n_samples = 12
    sub = rospy.Subscriber('imu/data', Imu, callback, (moving_average, n_samples))

    rospy.spin()

