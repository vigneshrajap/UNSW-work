
#include <row_detection/row_detection.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "row_detection");

  row_detection::RowDetection rd;

  rd.spin();

  return 0;

}

