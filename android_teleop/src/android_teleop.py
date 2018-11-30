
import rospy

from geometry_msgs.msg import Twist

import tf
from tf.transformations import euler_from_quaternion

import numpy as np

class AndroidTeleop(object):

  # Default constructor.
  def __init__(self):

    rospy.init_node('android_teleop')

    # Parse params.
    self.max_linear_vel = rospy.get_param('~max_linear_vel', 1.0)
    self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')

    # Set up publishers and subscribers.
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)

    self.tf_listener = tf.TransformListener()

    self.origin_inverse = np.array([])

    while not rospy.is_shutdown():
      try:
        trans, rot = self.tf_listener.lookupTransform('/odom', '/android', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException):
        continue

      roll, pitch, yaw = euler_from_quaternion(rot)
      cmd_vel = Twist()
      cmd_vel.linear.x = pitch/(np.pi/2)
      cmd_vel.angular.z = -roll/(np.pi/2)
      self.cmd_vel_pub.publish(cmd_vel)

      rospy.Rate(10).sleep()

