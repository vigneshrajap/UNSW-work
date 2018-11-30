
#include <row_detection/row_detection.h>

namespace row_detection
{

RowDetection::RowDetection() : duration_(5.0), time_between_assembles_(2.5)
{

  ros::NodeHandle private_nh("~");
  param_utils::ParamHelper param_helper(private_nh, ros::this_node::getName());
  
  param_helper.getParamWithInfo("duration", duration_);

  // Set up publishers.
  std::string lines_topic("lines");
  param_helper.getParamWithInfo("lines_topic", lines_topic);
  lines_pub_ = nh_.advertise<geometry_msgs::PoseArray>(lines_topic, 1);

  std::string markers_topic("line_markers");
  param_helper.getParamWithInfo("markers_topic", markers_topic);
  markers_pub_ = nh_.advertise<visualization_msgs::Marker>(markers_topic, 1);

  ros::NodeHandle line_detection_private_nh("~/line_detection");
  param_utils::ParamHelper line_detection_param_helper(line_detection_private_nh, ros::this_node::getName());

  std::string method;
  line_detection_param_helper.getParamWithInfo("method", method, true);
  
  if (method.compare("iterative") == 0)
  {

    double radius_min;
    double radius_max;
    int radius_samples;
    double angle_min;
    double angle_max;
    int angle_samples;
    int number_of_iterations;
    double distance_threshold;

    // Set up line detector.
    line_detection_param_helper.getParamWithInfo("radius_min", radius_min, true);
    line_detection_param_helper.getParamWithInfo("radius_max", radius_max, true);
    line_detection_param_helper.getParamWithInfo("radius_samples", radius_samples, true);
    line_detection_param_helper.getParamWithInfo("angle_min", angle_min, true);
    line_detection_param_helper.getParamWithInfo("angle_max", angle_max, true);
    line_detection_param_helper.getParamWithInfo("angle_samples", angle_samples, true);
    line_detection_param_helper.getParamWithInfo("number_of_iterations", number_of_iterations, true);
    line_detection_param_helper.getParamWithInfo("distance_threshold", distance_threshold, true);

    LineDetectionIterative line_detector_iterative;

    line_detector_iterative.setRadiusMin(radius_min);
    line_detector_iterative.setRadiusMax(radius_max);
    line_detector_iterative.setRadiusSamples(radius_samples);
    line_detector_iterative.setAngleMin(angle_min);
    line_detector_iterative.setAngleSamples(angle_samples);
    line_detector_iterative.setNumberOfIterations(number_of_iterations);
    line_detector_iterative.setDistanceThreshold(distance_threshold);

    line_detector_ = boost::shared_ptr<LineDetectionBase>(new LineDetectionIterative(line_detector_iterative));

  }
  else if (method.compare("ransac") == 0)
  {

    int number_of_lines;
    double distance_threshold;

    line_detection_param_helper.getParamWithInfo("number_of_lines", number_of_lines, true);
    line_detection_param_helper.getParamWithInfo("distance_threshold", distance_threshold, true);

    LineDetectionRANSAC line_detector_ransac;

    line_detector_ransac.setNumberOfLines(number_of_lines);
    line_detector_ransac.setDistanceThreshold(distance_threshold);

    line_detector_ = boost::shared_ptr<LineDetectionBase>(new LineDetectionRANSAC(line_detector_ransac));

  }
  else
  {
    ROS_ERROR("[row_detection::RowDetection]: <method> must be 'iterative' or 'ransac'");
  }

}

void RowDetection::spin()
{

  while (ros::ok())
  {

    ros::Duration(time_between_assembles_).sleep();

    cloud_ptr_t cloud(new cloud_t());
    point_cloud_assembler_.assemblePCL(*cloud, ros::Time::now() - ros::Duration(duration_), ros::Time::now(), laser_time_stamp_);

    line_detector_->setInputCloud(cloud);
    if (!line_detector_->compute())
    {
      continue;
    }

    publishLines(line_detector_->getLineSegments());
    publishMarkers(line_detector_->getLineSegments());

  }

}

void RowDetection::publishLines(const line_segment_vec_t & line_segments)
{

  // Lines represented as vector of poses, alternating between line starts and ends.
  geometry_msgs::PoseArray line_array;
  line_array.header.stamp = laser_time_stamp_;

  for (line_segment_vec_it_t line_segment = line_segments.begin(); line_segment != line_segments.end(); ++line_segment)
  {

    geometry_msgs::Pose line_start, line_end;

    line_start.position.x = (*line_segment)(0);
    line_start.position.y = (*line_segment)(1);

    line_end.position.x = (*line_segment)(3);
    line_end.position.y = (*line_segment)(4);

    line_array.poses.push_back(line_start);
    line_array.poses.push_back(line_end);

  }

  lines_pub_.publish(line_array);

}

void RowDetection::publishMarkers(const line_segment_vec_t & line_segments)
{

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "row_detection";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (line_segment_vec_it_t line_segment = line_segments.begin(); line_segment != line_segments.end(); ++line_segment)
  {

    geometry_msgs::Point line_start, line_end;
    line_start.x = (*line_segment)(0);
    line_start.y = (*line_segment)(1);
    line_end.x = (*line_segment)(3);
    line_end.y = (*line_segment)(4);

    marker.points.push_back(line_start);
    marker.points.push_back(line_end);

  }

  markers_pub_.publish(marker);

}

} // namespace row_detection

