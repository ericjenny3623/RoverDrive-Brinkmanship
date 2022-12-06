#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis
import tf

class IMUInterpreter:

    def __init__(self,
                 imuTopic: str,
                 imuDataType: str,
                 robotFrame: str,
                 verticalFrame: str,
                 disp: bool = False,
                 numSamples: int = 20) -> None:
        self.samples = []
        self.robot_frame = robotFrame
        self.vertical_frame = verticalFrame
        self._print = disp
        self._printCount = 0
        self.num_samples = numSamples
        self.br = tf.TransformBroadcaster()
        if imuDataType == "accel":
            cb = self.callback_accel
        elif imuDataType == "quat":
            cb = self.callback_quat
        self.sub = rospy.Subscriber(imuTopic, Imu, cb)
        print("IMU INTERP RUNNING +++++++++++++")

    def sendTF(self, q):
        self.br.sendTransform((0, 0, 0),
                              q,
                              rospy.Time.now(),
                              self.robot_frame,
                              self.vertical_frame)

    def callback_quat(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

        # Filter to remove noise (xsens might already do some of this!?)
        orientations = self.updateMovingAverage([roll, pitch, yaw])
        roll_ave = orientations[0]
        pitch_ave = orientations[1]
        yaw_ave = orientations[2]

        if self.checkDisp():
            print("Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
                roll * 180.0/np.pi,
                pitch * 180.0/np.pi,
                yaw * 180.0/np.pi))
            print("Filtered Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(
                roll_ave * 180.0/np.pi,
                pitch_ave * 180.0/np.pi,
                yaw_ave * 180.0/np.pi))

        # This might be backwards?
        self.sendTF(quaternion_from_euler(roll_ave, pitch_ave, yaw_ave))
        print("IMU INTERP PUBLISHING TF +++++++++++++")


    def callback_accel(self, msg):
        gx = msg.linear_acceleration.x
        gy = msg.linear_acceleration.y
        gz = msg.linear_acceleration.z
        gV = np.asarray((gz, -gx, gy)) # to account for weird axes of realsense -> ros
        gV = gV / np.linalg.norm(gV)

        # Filter to remove noise (xsens might already do some of this!?)
        gravityVector = self.updateMovingAverage(gV)

        Zunit = np.zeros(3)
        Zunit[2] = -1.

        cross = np.cross(Zunit, gravityVector)
        c = np.dot(gravityVector, Zunit)
        q = quaternion_about_axis(np.arccos(c), cross)

        if self.checkDisp():
            print("Filtered X: {:.2f} Y: {:.2f} Z: {:.2f}".format(
                gravityVector[0],
                gravityVector[1],
                gravityVector[2]))
        self.sendTF(q)

    def updateMovingAverage(self, sample):
        if (len(self.samples) > self.num_samples):
            self.samples.remove(self.samples[0])
        self.samples.append(sample)
        return np.mean(np.array(self.samples), axis=0)

    def checkDisp(self):
        self._printCount += 1
        if self._printCount % 100 == 0:
            self._printCount = 0
            return True
        else:
            return False


if __name__=="__main__":
    rospy.init_node('imu_interpreter')
    imuInterp = IMUInterpreter(imuTopic = "imu/data",
                               imuDataType = "quat", # quat / accel
                               robotFrame = "rs_front_link", # rs_front_link for testing only!
                               verticalFrame = "rs_front_link_upright", # Use upright / base_link on robot
                               disp = True)
    rospy.spin()

