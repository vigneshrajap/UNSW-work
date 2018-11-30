
#ifndef VITI_EKF_BASE_H_
#define VITI_EKF_BASE_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <param_utils/param_utils.h>

#include <vineyard_localisation/state_cov.h>

namespace vineyard_localisation
{

class EKFBase : public StateCov
{

public:

  EKFBase();

  virtual void init() = 0;

  virtual void spin() = 0;

protected:

  virtual void processModel(const Eigen::Vector4d & motion, double dt) = 0;

  void broadcastTransform();

  ros::NodeHandle node_handle_;

  tf::TransformBroadcaster tf_broadcaster_;

  std::string robot_base_frame_;

  ros::Time latest_time_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_BASE_H_

