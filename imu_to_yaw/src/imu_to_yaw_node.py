#! /usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def imuCallback(imu):
  quat = [imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z]
  roll, pitch, yaw = euler_from_quaternion(quat)
  rospy.loginfo('{} {} {}'.format(roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi))

def imuToYaw():

  rospy.init_node('imu_to_yaw')
  imu_sub = rospy.Subscriber('/android/imu', Imu, imuCallback)

  rospy.spin()

if __name__ == '__main__':
  imuToYaw()

