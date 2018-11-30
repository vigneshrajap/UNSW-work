
#ifndef POINT_CLOUD_ASSEMBLER_H_
#define POINT_CLOUD_ASSEMBLER_H_

#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>

#include <pcl_conversions/pcl_conversions.h>

namespace point_cloud_assembler
{

class PointCloudAssembler
{

public:

  PointCloudAssembler();

  bool assemblePCL(pcl::PointCloud<pcl::PointXYZ> & pcl_cloud, const ros::Time & begin, const ros::Time & end, ros::Time & time_stamp);

private:

  ros::NodeHandle node_handle_;

  ros::ServiceClient assemble_scans2_client_;

};

} // namespace point_cloud_assembler

#endif // #ifndef POINT_CLOUD_ASSEMBLER_H_

