
#include <vineyard_row_detection/vineyard_row_detection.h>
#include <vineyard_row_detection/line_association.h>

namespace vineyard_row_detection
{

VineyardRowDetection::VineyardRowDetection() : line_marker_count_(0)
{

  if (!ros::service::waitForService("assemble_scans2", ros::Duration(5.0)))
  {
    ROS_ERROR("[vineyard_row_detection::VineyardRowDetection]: assemble_scans2 service not available");
  }

  assemble_scans_client_ = node_handle_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  line_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("lines", 1);

  line_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("line_markers", 1);

  initLinesMarker();

  prev_assemble_time_ = ros::Time::now();

}

void VineyardRowDetection::spin()
{

  while (ros::ok())
  {

    ros::spinOnce();

    detectRows();

    ros::Rate(40.0).sleep();

  }

}

void VineyardRowDetection::initLinesMarker()
{

  lines_marker_.ns = "vineyard_rows";
  lines_marker_.id = line_marker_count_;
  lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
  lines_marker_.action= visualization_msgs::Marker::ADD;
  lines_marker_.header.frame_id = "/odom";
  lines_marker_.scale.x = 0.05;
  lines_marker_.color.r = 1.0;
  lines_marker_.color.g = 1.0;
  lines_marker_.color.b = 0.0;
  lines_marker_.color.a = 1.0;
  lines_marker_.lifetime = ros::Duration();

}

void VineyardRowDetection::detectRows()
{

  if ((ros::Time::now() - prev_assemble_time_).toSec() < 5.0)
  {
    return;
  }

  point_cloud_ptr_t pcl_cloud(new point_cloud_t());

  if (assemblePointCloud(pcl_cloud))
  {

    line_detector_.setInputCloud(pcl_cloud);

    Eigen::Matrix<float, 6, 10> lines_coefficients;
    line_detector_.detectLines(lines_coefficients);

    publishLines(lines_coefficients);
//    publishLineMarkers(lines_coefficients);

  }

}

bool VineyardRowDetection::assemblePointCloud(point_cloud_ptr_t & pcl_cloud)
{

  laser_assembler::AssembleScans2 assemble_scans_2_srv;

  assemble_scans_2_srv.request.begin = prev_assemble_time_;
  ros::Time time_now = ros::Time::now();
  assemble_scans_2_srv.request.end = time_now;
  prev_assemble_time_ = time_now; 

  if (assemble_scans_client_.call(assemble_scans_2_srv))
  {
    pcl::fromROSMsg(assemble_scans_2_srv.response.cloud, *pcl_cloud);

    if (pcl_cloud->points.size() < 2)
    {
      ROS_ERROR("[vineyard_row_detection::VineyardRowDetection]: not enough points in assembled point cloud");
      return false;
    }
    else
    {
      return true;
    }

  }
  else
  {
    ROS_ERROR("[vineyard_row_detection::VineyardRowDetection]: assemble_scans2 service call failed");
    return false;
  }

}

void VineyardRowDetection::publishLines(const Eigen::Matrix<float, 6, 10> & lines_coefficients)
{

  // Represent the line segments with pairs of poses. First pose in a pair is the start of a line, second pose in a pair is the end of a line.
  geometry_msgs::PoseArray lines;
  lines.header.frame_id = "/odom";

  for (int i = 0; i < lines_coefficients.cols(); ++i)
  {

    geometry_msgs::Pose line_start, line_end;

    line_start.position.x = lines_coefficients(0,i);
    line_start.position.y = lines_coefficients(1,i);
    line_start.position.z = lines_coefficients(2,i);

    line_end.position.x = lines_coefficients(3,i);
    line_end.position.y = lines_coefficients(4,i);
    line_end.position.z = lines_coefficients(5,i);

    lines.poses.push_back(line_start);
    lines.poses.push_back(line_end);

  }

  line_pub_.publish(lines);

}

// XXX Remove completely.
void VineyardRowDetection::publishLineMarkers(const Eigen::Matrix<float, 6, 10> & lines_coefficients)
{

    for (int i = 0; i < lines_coefficients.cols(); ++i)
    {

      geometry_msgs::Point line_start, line_end;

      line_start.x = lines_coefficients(0,i);
      line_start.y = lines_coefficients(1,i);
      line_start.z = lines_coefficients(2,i);

      line_end.x = lines_coefficients(3,i);
      line_end.y = lines_coefficients(4,i);
      line_end.z = lines_coefficients(5,i);

      lines_marker_.points.push_back(line_start);
      lines_marker_.points.push_back(line_end);

    }

    line_marker_pub_.publish(lines_marker_);

}

} // namespace vineyard_row_detection

