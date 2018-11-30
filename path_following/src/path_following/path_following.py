
import rospy

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

import tf
from tf.transformations import euler_from_quaternion

import copy

import numpy as np

import sys

class PathFollowing(object):

  def __init__(self, path=None):

    if path:
      self.path = path
      return

    use_gps_origin = rospy.get_param('~use_gps_origin', False)
    if use_gps_origin:
      self.getGpsOrigin()
    else:
      self.gps_origin_x = 0.0
      self.gps_origin_y = 0.0

    # Parse the path.
    self.initPath()

    self.publishPath()

    self.row_centre_start = dict()
    self.row_centre_end = dict()

    self.yaw = 0.44

    # Parse the command topic and set up cmd_vel publisher.
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)

    # Set up lines subscriber.
    self.lines_sub = rospy.Subscriber('/lines', PoseArray, self.linesCallback)

    # Parse the loop time.
    self.loop_time = rospy.get_param('~loop_time', 0.1)

    # Parse the tf_frames.
    self.robot_frame = rospy.get_param('~robot_frame', '/base_link')
    self.world_frame = rospy.get_param('~world_frame', '/odom')
    self.tf_listener = tf.TransformListener()

    # Initialise the controller.
    self.initController()

  def getGpsOrigin(self):

    gps_origin_sub = rospy.Subscriber('/gps_origin', Odometry, self.gpsOriginCallback)

    self.have_gps_origin = False

    while not self.have_gps_origin and not rospy.is_shutdown():
      rospy.loginfo('path_following.PathFollowing]: waiting for GPS origin')
      rospy.sleep(rospy.Duration(1.0))
  
  def gpsOriginCallback(self, gps_origin):
    
    self.gps_origin_x = gps_origin.pose.pose.position.x
    self.gps_origin_y = gps_origin.pose.pose.position.y
    self.have_gps_origin = True

  def initPath(self):
    # Parse the source of the path.
    if not rospy.has_param('~path_source'):
      rospy.logerr("[path_following.PathFollowing]: <path_source> not specified, must be 'param' or 'topic'")
      sys.exit(-1)
    else:
      path_source = rospy.get_param('~path_source')
      if not (path_source == 'param' or path_source == 'topic'):
        rospy.logerr("[path_following.PathFollowing]: <path_source> must be 'param' or 'topic'")
        sys.exit(-2)
      # Parse path from param.
      elif path_source == 'param':
        if not rospy.has_param('~path_param'):
          rospy.logerr("[path_following::PathFollowing]: <path_source> set to 'param', but <path_param> not specified")
          sys.exit(-3)
        else:
          path_param = rospy.get_param('~path_param')
          private_path_param = "~{}".format(path_param)
          if not rospy.has_param(private_path_param):
            rospy.logerr("[path_following::PathFollowing]: <path_param> set to '{}', but <{}> not specified".format(path_param, path_param))
            sys.exit(-4)
          else:
            self.path = rospy.get_param(private_path_param)
      elif path_source == "topic":
        pass

    # If an order is provided, the path parsed above is assumed to alternate between start and ends of the rows, and the order is used to generate the final path.
    if rospy.has_param('~path_order'):
      order = rospy.get_param('~path_order')
      order = [int(x) for x in order]
      unordered_path = self.path
      start_idx = 0
      end_idx = 1
      self.path = []
      for idx in order:
        self.path.append(copy.deepcopy(unordered_path[2*idx + start_idx]))
        self.path.append(copy.deepcopy(unordered_path[2*idx + end_idx]))
        start_idx, end_idx = end_idx, start_idx

    for i in xrange(len(self.path)):
      self.path[i]['x'] = self.path[i]['x'] - self.gps_origin_x
      self.path[i]['y'] = self.path[i]['y'] - self.gps_origin_y
      print self.path[i]

  def publishPath(self):

    path_pub = rospy.Publisher('/path_to_follow', Path)

    publish_path = Path()
    publish_path.header.frame_id = '/odom'
    publish_path.header.stamp = rospy.Time.now()

    for point in self.path:
      pose = PoseStamped()
      pose.header.frame_id = '/odom'
      pose.header.stamp = rospy.Time.now()
      pose.pose.position.x = point['x']
      pose.pose.position.y = point['y']
      publish_path.poses.append(pose)

    for i in xrange(10):
      path_pub.publish(publish_path)
      rospy.sleep(rospy.Duration(0.1))

  def initController(self):
    pass

  def calculateCmdVel(self):

    if len(self.path) <= 1:
      return Twist()

    # Lookup the robot to world transform.
    #t = self.tf_listener.getLatestCommonTime(self.robot_frame, self.world_frame)
    try:
      trans, rot = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      return Twist()
    roll, pitch, yaw = euler_from_quaternion(rot)

    self.yaw = yaw

    # Calculate the linear and angular errors to the path.
    linear_error, angular_error = self.calculatePathErrors(trans[0], trans[1], yaw)

    # Calculate the distance to the next point on the path.
    dx_next_point = self.path[1]['x'] - trans[0]
    dy_next_point = self.path[1]['y'] - trans[1]
    dist_next_point = np.sqrt(dx_next_point**2 + dy_next_point**2)

    # Calculate the change in angle at the next point on the path.
#    dx_current_path = self.path[1]['x'] - self.path[0]['x']
#    dy_current_path = self.path[1]['y'] - self.path[0]['y']
#    angle_current_path = np.arctan2(dy_current_path, dx_current_path)

#    if len(self.path) >= 3:
#      dx_next_path = self.path[2]['x'] - self.path[1]['x']
#      dy_next_path = self.path[2]['y'] - self.path[1]['y']
#      angle_next_path = np.arctan2(dy_next_path, dx_next_path)
#    else:
#      angle_next_path = 0.0

#    delta_angle_path = angle_next_path - angle_current_path

#    while delta_angle_path >= np.pi:
#      delta_angle_path -= 2.0*np.pi
#    while delta_angle_path < -np.pi:
#      delta_angle_path += 2.0*np.pi

    # A waypoint is coming up.
    if dist_next_point < 3.0: #and np.fabs(delta_angle_path) > np.pi/4.0:
      linear = 0.5
      angular = self.calculateAngularControlEffort(linear, linear_error, angular_error)
    # We have a large angular error.
    elif np.fabs(angular_error) > np.pi/4.0:
      linear = 0.0
      angular = -np.sign(angular_error)
    else:
      linear = 1.0
      angular = self.calculateAngularControlEffort(linear, linear_error, angular_error)

    if dist_next_point < 1.0:
      self.path.pop(0)
      self.row_centre_start = dict()
      self.row_centre_end = dict()

    cmd_vel = Twist()
    cmd_vel.linear.x = linear
    cmd_vel.angular.z = angular

    return cmd_vel

  def calculatePathErrors(self, x, y, th):

    if self.row_centre_start and self.row_centre_end:
      path0 = self.row_centre_start
      path1 = self.row_centre_end
    else:
      path0 = self.path[0]
      path1 = self.path[1]

    line_start = np.array([path0['x'], path0['y']])
    line_end = np.array([path1['x'], path1['y']])

    robot_pose = np.array([x, y])

    line_point = line_start
    line_dir = line_end - line_start
    line_dir = line_dir/np.linalg.norm(line_dir)

    delta_robot_pose = line_point - robot_pose

    dist_to_line = np.linalg.norm(delta_robot_pose - np.dot(delta_robot_pose, line_dir)*line_dir)
    linear_error = -np.sign(np.cross(delta_robot_pose, line_dir))*dist_to_line
    angular_error = th - np.arctan2(line_dir[1], line_dir[0])

    # Normalise the angular error.
    while angular_error >= np.pi:
      angular_error -= 2.0*np.pi
    while angular_error < -np.pi:
      angular_error += 2.0*np.pi

    return linear_error, angular_error
  
  def calculateAngularControlEffort(self, linear_vel, linear_error, angular_error):
    k1 = 1.0
    k2 = 1.0
    return -k2*np.cos(angular_error)*(np.sin(angular_error) - k1*linear_error) - k1*linear_vel*np.tan(angular_error)
  
  def linesCallback(self, lines):

    self.row_centre_start = dict()
    self.row_centre_end = dict()

    # Require 2 lines to be detected.
    if len(lines.poses) != 4:
      return

    row_centre_start = dict()
    row_centre_end = dict()
    # Take the average of the lines as the centre of the row.
    row_centre_start['x'] = (lines.poses[0].position.x + lines.poses[2].position.x)/2.0
    row_centre_start['y'] = (lines.poses[0].position.y + lines.poses[2].position.y)/2.0
    row_centre_end['x'] = (lines.poses[1].position.x + lines.poses[3].position.x)/2.0
    row_centre_end['y'] = (lines.poses[1].position.y + lines.poses[3].position.y)/2.0

    # Require the row centre line to be a certain length.
    dx_row_centre = row_centre_start['x'] - row_centre_end['x']
    dy_row_centre = row_centre_start['y'] - row_centre_end['y']
    dist_row_centre = np.sqrt(dx_row_centre**2 + dy_row_centre**2)

    if dist_row_centre < 1.5:
      return

    row_centre_angle = np.arctan2(row_centre_end['y'] - row_centre_start['y'], row_centre_end['x'] - row_centre_start['x'])
    row_centre_angle_pi = row_centre_angle + np.pi

    while row_centre_angle >= np.pi:
      row_centre_angle -= 2.0*np.pi
    while row_centre_angle < -np.pi:
      row_centre_angle += 2.0*np.pi

    while row_centre_angle_pi >= np.pi:
      row_centre_angle_pi -= 2.0*np.pi
    while row_centre_angle_pi < -np.pi:
      row_centre_angle_pi += 2.0*np.pi

    # The centre of the row is in the same direction.
    if np.fabs(row_centre_angle - self.yaw) < np.deg2rad(10.0):
      self.row_centre_start = row_centre_start
      self.row_centre_end = row_centre_end
    # The centre of the row is in the opposite direction.
    elif np.fabs(row_centre_angle_pi - self.yaw) < np.deg2rad(10.0):
      self.row_centre_start = row_centre_end
      self.row_centre_end = row_centre_start

  def spin(self):

    while not rospy.is_shutdown():
      cmd_vel = self.calculateCmdVel()
      self.cmd_vel_pub.publish(cmd_vel)
      rospy.sleep(rospy.Duration(self.loop_time))

