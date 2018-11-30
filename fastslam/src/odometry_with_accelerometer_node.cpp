
#include <fastslam/odometry_with_accelerometer.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "odometry_with_acclerometer");

  fastslam::OdometryWithAccelerometer odometry_with_acclerometer;

  odometry_with_acclerometer.spin();

  return 0;

}
