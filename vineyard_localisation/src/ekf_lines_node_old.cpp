
#include <vineyard_localisation/ekf_lines.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "ekf_lines");

  vineyard_localisation::EKFLines ekf_lines;

  ekf_lines.spin();

  return 0;

}

