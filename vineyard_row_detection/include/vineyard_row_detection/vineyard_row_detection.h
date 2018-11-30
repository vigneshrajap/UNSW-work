
#ifndef VINEYARD_ROW_DETECTION_H_
#define VINEYARD_ROW_DETECTION_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <laser_assembler/AssembleScans2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <vineyard_row_detection/line_detector.h>

namespace vineyard_row_detection
{

class VineyardRowDetection
{

public:

  VineyardRowDetection();

  void spin();

private:

  void initLinesMarker();

  void detectRows();

  bool assemblePointCloud(point_cloud_ptr_t & pcl_cloud);

  void publishLines(const Eigen::Matrix<float, 6, 10> & lines_coefficients);

  void publishLineMarkers(const Eigen::Matrix<float, 6, 10> & lines_coefficients);

  ros::NodeHandle node_handle_;

  ros::Publisher line_pub_;

  ros::Publisher line_marker_pub_;

  ros::ServiceClient assemble_scans_client_;

  visualization_msgs::Marker lines_marker_;

  LineDetector line_detector_;

  ros::Time prev_assemble_time_;

  int line_marker_count_;

};

} // namespace vineyard_row_detection

#endif // #ifndef VINEYARD_ROW_DETECTION_H_

