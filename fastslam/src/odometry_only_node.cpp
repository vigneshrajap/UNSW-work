
#include <fastslam/odometry_only.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "odometry_only");

  fastslam::OdometryOnly odometry_only;

  odometry_only.spin();

  return 0;

}

