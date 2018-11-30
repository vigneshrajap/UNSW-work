
#include <fastslam/odometry_with_gps.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "odometry_with_gps");

  fastslam::OdometryWithGPS odometry_with_gps;

  odometry_with_gps.spin();

  return 0;

}

