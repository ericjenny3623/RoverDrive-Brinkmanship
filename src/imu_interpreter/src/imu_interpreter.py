#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis
import tf


def callback_quat(msg, args):
    qx = msg.orientation.x
    qy = msg.orientation.y
    qz = msg.orientation.z
    qw = msg.orientation.w
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    # Filter to remove noise (xsens might already do some of this!?)
    moving_average = args[0]
    n_samples = args[1]
    br = args[2]
    if (len(moving_average) > n_samples):
        moving_average.remove(moving_average[0])
    moving_average.append([roll, pitch, yaw])
    orientations = np.mean(np.array(moving_average), axis=0)
    roll_ave = orientations[0]
    pitch_ave = orientations[1]
    yaw_ave = orientations[2]

    if np.random.randint(100) == 0:
        print("Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
            roll * 180.0/np.pi,
            pitch * 180.0/np.pi,
            yaw * 180.0/np.pi))
        print("Filtered Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
            roll_ave * 180.0/np.pi,
            pitch_ave * 180.0/np.pi,
            yaw_ave * 180.0/np.pi))

    # This might be backwards?
    br.sendTransform((0, 0, 0),
                    quaternion_from_euler(roll_ave, pitch_ave, yaw_ave),
                    rospy.Time.now(),
                    "base_link",
                    "upright")

def callback_accel(msg, args):
    gx = msg.linear_acceleration.x
    gy = msg.linear_acceleration.y
    gz = msg.linear_acceleration.z
    gV = np.asarray((gz, -gx, gy)) # to account for weird axes of realsense -> ros
    gV = gV / np.linalg.norm(gV)

    # Filter to remove noise (xsens might already do some of this!?)
    moving_average = args[0]
    n_samples = args[1]
    br = args[2]
    if (len(moving_average) > n_samples):
        moving_average.remove(moving_average[0])
    moving_average.append(gV)
    gravityVector = np.mean(np.array(moving_average), axis=0)

    Zunit = np.zeros(3)
    Zunit[2] = -1.

    cross = np.cross(Zunit, gV)
    c = np.dot(gV, Zunit)
    q = quaternion_about_axis(np.arccos(c), cross)

    if np.random.randint(100) == 0:
        print("Filtered X: {:.2f} Y: {:.2f} Z: {:.2f}".format(
            gravityVector[0],
            gravityVector[1],
            gravityVector[2]))
        # print(q)

    # This might be backwards?
    br.sendTransform((0, 0, 0),
                        q,
                        rospy.Time.now(),
                        "base_link",
                        "upright")


if __name__=="__main__":
    rospy.init_node('imu_interpreter')
    moving_average = []
    n_samples = 12
    br = tf.TransformBroadcaster()
    use_quat = False
    sub = rospy.Subscriber('imu/data', Imu, callback_accel, (moving_average, n_samples, br))

    rospy.spin()

