
import rospy

from geometry_msgs.msg import PoseArray

import tf
from tf.transformations import euler_from_quaternion

import numpy as np

class RowEndpointDetection(object):

  def __init__(self):

    # Parse params.
    lines_topic = rospy.get_param('~lines_topic', '/state_lines')
    path_topic = rospy.get_param('~path_topic', '/path')
    self.robot_base_frame = rospy.get_param('~robot_base_frame', '/base_link')
    self.world_frame = rospy.get_param('~world_frame', '/odom')

    # Set up subscribers and publishers.

    # Subscribe to the lines that have been detected.
    self.lines_sub = rospy.Subscriber(lines_topic, PoseArray, self.linesCallback)

    # Set up tf listener.
    self.tf_listener = tf.TransformListener()

  def linesCallback(self, lines):

    # Array alternates between starts and end of detected lines.
    # Find the points on the lines and the direction that the line is going in.
    line_points = [];
    line_directions = [];
    for line_idx in xrange(0, len(lines.poses), 2):
 
      line_start = lines.poses[line_idx].position
      line_end = lines.poses[line_idx + 1].position

      # Parametrise the line with point and direction.
      line_point = np.array([line_start.x, line_start.y, 0.0])
      line_direction = np.array([line_end.x - line_start.x, line_end.y - line_start.y, 0.0])

      # Make sure all of the lines are pointing in the same direction.
      if line_idx > 0 and np.dot(line_directions[0], line_direction) < 0:
        line_point = line_point + line_direction
        line_direction *= -1

      line_points.append(line_point)
      line_directions.append(line_direction)

    # Find the signed distance from each point to the origin.
    origin = np.array([0.0, 0.0, 0.0])
    dists = []
    for line_point, line_direction in zip(line_points, line_directions):

      # Find which side of the origin the line is on.
      delta_origin = origin - line_point

      line_direction_unit = line_direction/np.linalg.norm(line_direction)

      # Find the distance from the robot to the line.
      dist = np.linalg.norm(delta_origin - (np.dot(delta_origin, line_direction_unit)*line_direction_unit))

      cross_product = np.cross(line_direction, delta_robot_2d)
      
      dists.append(dist*cross_product[2])

    # Order the lines based on their distances from the origin.
    line_zip = zip(dists, line_points, line_directions)
    line_zip = sorted(line_zip)

    # Reconstruct the PoseArray with the ordered line.
    ordered_lines = PoseArray()
    for dist, line_point, line_direction in line_zip:

      line_start = line_point
      line_end = line_point + line_direction

      line_start_pose = Pose()
      line_start_pose.position.x = line_start[0]
      line_start_pose.position.y = line_start[1]
      line_start_pose.position.z = 0.0

      line_end_pose = Pose()
      line_end_pose.position.x = line_start[0]
      line_end_pose.position.y = line_start[1]
      line_end_pose.position.z = 0.0

      ordered_lines.poses.append(line_start_pose)
      ordered_lines.poses.append(line_end_pose)

  # Get the most recent yaw of the robot.
  def getRobot2d(self):

    try:
      latest_common_time = self.tf_listener.getLatestCommonTime(self.robot_base_frame, self.world_frame)
      position, quaternion = self.tf_listener.lookupTransform(self.world_frame, self.robot_base_frame, latest_common_time)
      roll, pitch, yaw = euler_from_quaternion(quaternion)
      return position[0], position[1], yaw
    except tf.ConnectivityException:
      rospy.logwarn('[row_endpoint_detection.RowEndpointDetection]: unable to lookup transform between {} and {}'.format(self.robot_base_frame, self.world_frame))

