
#include <vineyard_localisation/ekf_base.h>

namespace vineyard_localisation
{

EKFBase::EKFBase() : robot_base_frame_("/base_link")
{

  ros::NodeHandle private_node_handle("~");
  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());
  param_helper.getParamWithInfo("robot_base_frame", robot_base_frame_);

}

void EKFBase::broadcastTransform()
{

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(tx(), ty(), tz()));
  transform.setRotation(tf::Quaternion(qx(), qy(), qz(), qw()));

  if (latest_time_ != ros::Time())
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, latest_time_, "/odom", robot_base_frame_));
  }

}

} // namespace vineyard_localisation

