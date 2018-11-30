
import random

import rospy

from nav_msgs.msg import Odometry

class OdometryNoise(object):

  def __init__(self):

    rospy.init_node('odometry_noise')

    # Parse params.
    self.position_sigma = rospy.get_param('~position_sigma', 0.0)
    self.twist_sigma = rospy.get_param('~twist_sigma', 0.0)
    sensor_odom_topic = rospy.get_param('~sensor_odom_topic', 'odom')
    noisy_odom_topic = rospy.get_param('~noisy_odom_topic', 'noisy_odom')

    # Set up publishers and subscribers.
    self.odom_sub = rospy.Subscriber(sensor_odom_topic, Odometry, self.odomCallback)
    self.odom_pub = rospy.Publisher(noisy_odom_topic, Odometry)

    rospy.spin()

  def odomCallback(self, odom):

    odom.pose.pose.position.x += random.gauss(0.0, self.position_sigma) 
    odom.pose.pose.position.y += random.gauss(0.0, self.position_sigma) 
    odom.pose.pose.position.z += random.gauss(0.0, self.position_sigma) 

    cov = list(odom.pose.covariance)

    cov[ 0] += self.position_sigma**2
    cov[ 7] += self.position_sigma**2
    cov[14] += self.position_sigma**2

    odom.pose.covariance = tuple(cov)

    self.odom_pub.publish(odom)

