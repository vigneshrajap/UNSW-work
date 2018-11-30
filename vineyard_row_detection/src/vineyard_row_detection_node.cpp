
#include <vineyard_row_detection/vineyard_row_detection.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "vineyard_row_detection");

  vineyard_row_detection::VineyardRowDetection vineyard_row_detector;

  vineyard_row_detector.spin();

  return 0;

}

