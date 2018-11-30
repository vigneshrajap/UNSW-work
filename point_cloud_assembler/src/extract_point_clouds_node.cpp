
#include <point_cloud_assembler/point_cloud_assembler.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "extract_point_clouds");

  ros::NodeHandle private_node_handle_("~");

  std::string pcd_base("cloud");
  private_node_handle_.getParam("pcd_base", pcd_base);

  double duration = 5.0;
  private_node_handle_.getParam("duration", duration);

  point_cloud_assembler::PointCloudAssembler assembler;

  int i = 0;

  while (ros::ok())
  {

    ros::Duration(5.0).sleep();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    ros::Time time_stamp;
    assembler.assemblePCL(cloud, ros::Time::now() - ros::Duration(5.0), ros::Time::now(), time_stamp);

    if (cloud.points.size() == 0)
    {
      ROS_WARN("[point_cloud_assembler::ExtractPointClouds]: received empty point cloud");
      continue;
    }

    std::stringstream ss;
    ss << pcd_base << i++ << ".pcd";
    pcl::io::savePCDFileBinary(ss.str(), cloud);

  }

  return 0;

}

