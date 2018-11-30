
import rospy

from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Path

import tf
from tf.transformations import euler_from_quaternion

import ast

from math import acos, atan2, cos, sin, tan

import numpy as np

class GeneralPathFollowing(object):

  def __init__(self):
    
    # Initialise an empty path.
    self.path = []
    self.path_types = []

    self.visited = []

    # Get the topic that lines are published on and subscribe to it.
    lines_topic = rospy.get_param('~lines_topic', '/state_lines')
    self.lines_sub = rospy.Subscriber(lines_topic, PoseArray, self.lines_callback)

    # Get the topic to publish the path on, primarily for visualisation purposes.
    path_topic = rospy.get_param('~path_topic', 'path_graph')
    self.path_pub = rospy.Publisher(path_topic, Path)

    # Get the command topic and advertise it.
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
    self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist)

    # Get the follow path period and  set up the timer for following the path.
    path_follow_period = rospy.get_param('~path_follow_period', 0.05)
    self.path_follow_timer = rospy.Timer(rospy.Duration(path_follow_period), self.follow_path)

    # Parse the robot base and world frames.
    self.robot_base_frame = rospy.get_param('~robot_base_frame', '/base_link')
    self.world_frame = rospy.get_param('~world_frame', '/odom')

    # Set up the tf listener.
    self.tf_listener = tf.TransformListener()

  def follow_path(self, event):

    # We either don't have a path or have reach the end of the path.
    if len(self.path) <= 1:
      return

    # Get the robot's current position.
    x, y, yaw, tf_success = self.robot_2d()
    
    # The transform from the world frame to the robot base frame is not available.
    if not tf_success:
      return

    # Calculate the linear and angular errors.
    linear_error, angular_error = self.calculate_path_errors(x, y, yaw)

    # Calculate the control effort.
    cmd_vel = self.calculate_control_effort(linear_error, angular_error)

    # Publish the command velocity.
    self.cmd_vel_pub.publish(cmd_vel)

  def calculate_path_errors(self, x, y, yaw):
    
    # The robot's position and direction.
    robot_point = np.array([x, y, 0.0])
    robot_direction = np.array([cos(yaw), sin(yaw), 0.0])

    # Parametrise the line in terms of point and direction.
    line_point = self.path[0]
    line_direction = self.path[1] - self.path[0]

    # We have reached the next point on the path.
    if np.linalg.norm(self.path[1] - robot_point) < 0.25:
      if self.path_types[0] == 'row':
        self.visited.append(self.path_ids[self.current_path_idx])
      self.path = self.path[1:]
      self.path_types = self.path_types[1:]

    # Have the line point the same direction as the robot.
    #if (np.dot(robot_direction, line_direction) < 0):
    #  line_point += line_direction
    #  line_direction *= -1.0;

    # Normalise the robot and line directions.
    robot_direction /= np.linalg.norm(robot_direction)
    line_direction /= np.linalg.norm(line_direction)

    # Calculate the linear error.
    delta_robot = line_point - robot_point
    dist = np.linalg.norm(delta_robot - np.dot(delta_robot, line_direction)*line_direction);
    cross_product = np.cross(delta_robot, line_direction)
    linear_error = np.sign(cross_product[2])*dist

    # Calculate the angular error.
    angular_error = atan2(robot_direction[1], robot_direction[0]) - atan2(line_direction[1], line_direction[0])
    while angular_error > np.pi:
      angular_error -= 2.0*np.pi
    while angular_error <= -np.pi:
      angular_error += 2.0*np.pi

    return linear_error, angular_error

  def calculate_control_effort(self, linear_error, angular_error):

    k_p = 1.0
    k_d = 1.0

    cmd_vel = Twist()

    if abs(angular_error) < 45.0*np.pi/180.0:
      linear_vel = 0.75
      cmd_vel.linear.x = linear_vel
      cmd_vel.angular.z = linear_vel*cos(angular_error)**3*(-k_d*tan(angular_error) - k_p*linear_error)
    else:
      cmd_vel.linear.x = 0.0
      cmd_vel.angular.z = -np.sign(angular_error)*1.0

    return cmd_vel

  def robot_2d(self):

    try:
      trans, rot = self.tf_listener.lookupTransform(self.world_frame, self.robot_base_frame, rospy.Time(0))
      row, pitch, yaw = euler_from_quaternion(rot)
      return trans[0], trans[1], yaw, True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      return None, None, None, False

  def lines_callback(self, lines):

    if len(self.path_types) > 0 and self.path_types[0] == 'transition':
      return

    # Clear the current lines.
    self.line_points = []
    self.line_directions = []
    self.line_ids = []

    # Get the direction of the robot.
    x, y, yaw, tf_success = self.robot_2d()
    
    # Transform from world frame to robot base frame is unavailable.
    if not tf_success:
      return

    robot_direction = np.array([cos(yaw), sin(yaw), 0.0])

    # Fill in the path with the poses in the PoseArray.
    # Points alternate between the starts and ends of lines.
    for pose_idx in xrange(0, len(lines.poses), 2):

      # Get the 2 points out of the array.
      pose_1 = lines.poses[pose_idx]
      pose_2 = lines.poses[pose_idx + 1]

      point_1 = np.array([pose_1.position.x, pose_1.position.y, 0.0])
      point_2 = np.array([pose_2.position.x, pose_2.position.y, 0.0])

      # Orient the line so that it is facing the same direction as the robot.
      line_point = point_1
      line_direction = point_2 - point_1

      if np.dot(robot_direction, line_direction) < 0.0:
        line_point += line_direction
        line_direction *= -1.0

      # Add the line to the list.
      # It is assumed that the order that lines are published in will not change, such that the id of line will always be the same.
      self.line_points.append(line_point)
      self.line_directions.append(line_direction)
      self.line_ids.append(pose_idx/2)

    # Order the lines based on their signed distance to the origin.
    self.order_lines()

    # Convert the rows into paths by taking the average of the starts and ends.
    self.rows_to_paths()

    self.update_path(x, y, yaw)

  def order_lines(self):

    # Calculate the signed distance from each line to the origin.
    origin = np.array([0.0, 0.0, 0.0])
    dists = []
    for line_point, line_direction in zip(self.line_points, self.line_directions):

      # Find which side of the origin the line is on.
      delta_origin = line_point - origin

      line_direction_unit = line_direction/np.linalg.norm(line_direction)

      # Make sure all the lines are pointing in the same direction.
      if line_direction_unit[0] > 0:
        line_direction_unit *= -1.0

      # Find signed the distance from the origin to the line.
      unsigned_dist = np.linalg.norm(delta_origin - (np.dot(delta_origin, line_direction_unit)*line_direction_unit))

      cross_product = np.cross(line_direction_unit, delta_origin)

      signed_dist = np.sign(cross_product[2])*unsigned_dist

      dists.append(signed_dist)

    print dists

    # Sort the lines based on their signed distances to the origin.
    sorted_lines = sorted(zip(dists, self.line_points, self.line_directions, self.line_ids))
    self.line_points     = [line_point     for dist, line_point, line_direction, line_id in sorted_lines]
    self.line_directions = [line_direction for dist, line_point, line_direction, line_id in sorted_lines]
    self.line_ids        = [line_id        for dist, line_point, line_direction, line_id in sorted_lines]

  def rows_to_paths(self):

    self.path_points = []
    self.path_directions = []
    self.path_ids = []

    # Add path before the first row, so that the robot can travel on the outside of the row.
    first_row_line_direction_unit = self.line_directions[0]/np.linalg.norm(self.line_directions[0])
    # Rotate 90 degrees counter-clockwise and assume 3 metres spacing between rows.
    #first_row_delta = np.array([first_row_line_direction_unit[1], -first_row_line_direction_unit[0], 0.0])
    first_row_delta = np.array([0.0, 1.0, 0.0])

    # Add the path before the first row.
    self.path_points.append(self.line_points[0] + 1.5*first_row_delta)
    self.path_directions.append(self.line_directions[0])
    self.path_ids.append([-1, self.line_ids[0]])

    for line_idx in xrange(0, len(self.line_points) - 1):

      # Find the start and end of 2 neighbouring lines.
      line_start_1 = self.line_points[line_idx]
      line_end_1   = self.line_points[line_idx] + self.line_directions[line_idx]
      line_id_1    = self.line_ids[line_idx]
      line_start_2 = self.line_points[line_idx + 1]
      line_end_2   = self.line_points[line_idx + 1] + self.line_directions[line_idx + 1]
      line_id_2    = self.line_ids[line_idx + 1]

      path_start = (line_start_1 + line_start_2)/2.0
      path_end   = (line_end_1   + line_end_2  )/2.0

      self.path_points.append(path_start)
      self.path_directions.append(path_end - path_start)
      self.path_ids.append([line_id_1, line_id_2])

    # Add path after the last row, so that the robot can travel on the outside of the row.
    end_idx = len(self.line_points) - 1
    last_row_line_direction_unit = self.line_directions[end_idx]/np.linalg.norm(self.line_directions[end_idx])
    # Rotate 90 degrees XXX clockwise and assume 3 metres spacing between rows.
    #last_row_delta = np.array([-last_row_line_direction_unit[1], last_row_line_direction_unit[0], 0.0])
    last_row_delta = np.array([0.0, -1.0, 0.0])

    # Add the path afet the last row.
    self.path_points.append(self.line_points[end_idx] + 1.5*last_row_delta)
    self.path_directions.append(self.line_directions[end_idx])
    self.path_ids.append([self.line_ids[end_idx], -1])

  # Update the path.
  def update_path(self, x, y, yaw):

    self.current_path_idx = self.find_current_path_idx(x, y, yaw)

    self.path = []
    self.path_types = []
    mod_2 = 0
    for path_idx in xrange(self.current_path_idx, len(self.path_points)):
      path_direction_unit = self.path_directions[path_idx]/np.linalg.norm(self.path_directions[path_idx])
      if mod_2 == 0:
#        if not self.has_been_visited(self.path_ids[path_idx]):
        self.path.append(self.path_points[path_idx] - 1.0*path_direction_unit)
        self.path_types.append('row')
        self.path.append(self.path_points[path_idx] + self.path_directions[path_idx] + 2.0*path_direction_unit)
        self.path_types.append('transition')
      else:
        self.path.append(self.path_points[path_idx] + self.path_directions[path_idx] + 2.0*path_direction_unit)
        self.path_types.append('row')
        self.path.append(self.path_points[path_idx] - 1.0*path_direction_unit)
        self.path_types.append('transition')
      mod_2 = (mod_2 + 1) % 2

#    if current_row_idx < 0:
#      self.path = []
#      return

#    self.construct_path(x, y, yaw, current_row_idx, 1)

    # Publish the path.
    self.publish_path()

  # Find the index of the path that the robot is currently on.
  def find_current_path_idx(self, x, y, yaw):

    robot_pose = np.array([x, y, 0.0])

    min_idx = -1
    min_dist = np.Inf

    for path_point, path_direction, path_idx in zip(self.path_points, self.path_directions, xrange(0, len(self.path_points))):

      # Normalise the direction.
      path_direction_unit = path_direction/np.linalg.norm(path_direction)

      # Calculate the distance from the robot to the line.
      delta_robot = path_point - robot_pose
      dist = np.linalg.norm(delta_robot - np.dot(delta_robot, path_direction_unit)*path_direction_unit)

      if dist < min_dist:
        min_idx = path_idx
        min_dist = dist

    return min_idx

  def has_been_visited(self, path_id):

    for visited_id in self.visited:
      if path_id[0] == visited_id[0] or path_id[1] == visited_id[1]:
        return True

    return False

  # Construct the path for the robot to follow.
  def construct_path(self, x, y, yaw, start_row_idx, row_change):

    if row_change > 0:
      row_indices = xrange(start_row_idx, len(self.path_nodes)/2, row_change)
    elif row_change < 0:
      row_indices = xrange(start_row_idx, -1, row_change)

    robot_direction = np.array([cos(yaw), sin(yaw), 0.0])

    # Rearrange the starts and ends of the rows so that the rows are oriented the same way as the robot.
    line_starts = [];
    line_ends = [];
    for line_idx in xrange(0, len(self.path_nodes), 2):
      line_direction = np.array([self.path_nodes[line_idx + 1]['x'] - self.path_nodes[line_idx]['x'], self.path_nodes[line_idx + 1]['y'] - self.path_nodes[line_idx]['y'], 0.0])
      if np.dot(robot_direction, line_direction) > 0.0:
        line_starts.append(np.array([self.path_nodes[line_idx]['x'], self.path_nodes[line_idx]['y'], 0.0]))
        line_ends.append(np.array([self.path_nodes[line_idx + 1]['x'], self.path_nodes[line_idx + 1]['y'], 0.0]))
      else:
        line_starts.append(np.array([self.path_nodes[line_idx + 1]['x'], self.path_nodes[line_idx + 1]['y'], 0.0]))
        line_ends.append(np.array([self.path_nodes[line_idx]['x'], self.path_nodes[line_idx]['y'], 0.0]))

    start_or_end = 'start'

    self.path = []
    for row_idx in row_indices:
      if start_or_end == 'start':
        self.path.append(line_starts[row_idx])
        self.path.append(line_ends[row_idx])
        start_or_end = 'end'
      else:
        self.path.append(line_ends[row_idx])
        self.path.append(line_starts[row_idx])
        start_or_end = 'start'

  def publish_path(self):

    # Initialise the path.
    path = Path()
    path.header.frame_id = '/odom'
    time_now = rospy.Time.now()
    path.header.stamp = time_now

    for point in self.path:

      pose = PoseStamped()

      pose.header.frame_id = '/odom'
      pose.header.stamp = time_now

      pose.pose.position.x = point[0]
      pose.pose.position.y = point[1]

      path.poses.append(pose)

    self.path_pub.publish(path)

