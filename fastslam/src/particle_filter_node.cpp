
#include <fastslam/particle_filter_ros.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "particle_filter");

  fastslam::ParticleFilterROS particle_filter;
  
  particle_filter.spin(); 

  return 0;

}

