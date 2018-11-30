
#ifndef ROW_DETECTION_H_
#define ROW_DETECTION_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <param_utils/param_utils.h>

#include <point_cloud_assembler/point_cloud_assembler.h>

#include <row_detection/line_detection_iterative.h>
#include <row_detection/line_detection_ransac.h>

namespace row_detection
{

class RowDetection
{

public:

  RowDetection(); 

  void spin();

private:

  void publishLines(const line_segment_vec_t & line_segments);

  void publishMarkers(const line_segment_vec_t & line_segments);

  ros::NodeHandle nh_;

  ros::Publisher lines_pub_;

  ros::Publisher markers_pub_;

  point_cloud_assembler::PointCloudAssembler point_cloud_assembler_;

  boost::shared_ptr<LineDetectionBase> line_detector_;

  double duration_;

  double time_between_assembles_;

  ros::Time laser_time_stamp_;

};

} // namespace row_detection

#endif // #ifndef ROW_DETECTION_H_

