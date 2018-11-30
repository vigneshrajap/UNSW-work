
#include <vineyard_localisation/ekf_gps.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "ekf_gps");

  vineyard_localisation::EKFGPS ekf_gps;

  ekf_gps.init();

  ekf_gps.spin();

  return 0;

}

