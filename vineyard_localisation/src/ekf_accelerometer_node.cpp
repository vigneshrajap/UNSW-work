
#include <vineyard_localisation/ekf_accelerometer.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "ekf_accelerometer");

  vineyard_localisation::EKFAccelerometer ekf_acc;

  ekf_acc.spin();

  return 0;

}

