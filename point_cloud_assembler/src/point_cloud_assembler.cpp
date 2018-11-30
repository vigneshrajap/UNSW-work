
#include <point_cloud_assembler/point_cloud_assembler.h>

namespace point_cloud_assembler
{

PointCloudAssembler::PointCloudAssembler()
{

  if (!ros::service::waitForService("assemble_scans2", ros::Duration(5.0)))
  {
    ROS_ERROR("[point_cloud_assembler::PointCloudAssembler]: assemble_scans2 service not available");
  }

  assemble_scans2_client_ = node_handle_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

}

bool PointCloudAssembler::assemblePCL(pcl::PointCloud<pcl::PointXYZ> & pcl_cloud, const ros::Time & begin, const ros::Time & end, ros::Time & time_stamp)
{
  
  laser_assembler::AssembleScans2 assemble_scans2_srv;
  assemble_scans2_srv.request.begin = begin;
  assemble_scans2_srv.request.end = end;

  if (assemble_scans2_client_.call(assemble_scans2_srv))
  {
    pcl::fromROSMsg(assemble_scans2_srv.response.cloud, pcl_cloud);
    time_stamp = assemble_scans2_srv.response.cloud.header.stamp;
    return true;
  }
  else
  {
    ROS_ERROR("[point_cloud_assembler::PointCloudAssembler]: assemble_scans2 service call failed");
    return false;
  }

}

} // namespace point_cloud_assembler

