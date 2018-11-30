
#include <fastslam/state_3d.h>

namespace fastslam
{

State3D::State3D()
{

  state_ = Vector7d::Zero();

  Eigen::Quaterniond identity_quat = Eigen::Quaterniond::Identity();

  state_(3) = identity_quat.w();
  state_(4) = identity_quat.x();
  state_(5) = identity_quat.y();
  state_(6) = identity_quat.z();

}

void State3D::processModel(const Eigen::Vector4d & motion, double dt)
{

  // Get rotational component of state as a quaternion.
  Eigen::Quaterniond rot_state = quat();

  // Get translational component of motion.
  Eigen::Vector3d trans_motion_local = Eigen::Vector3d(motion(0)*dt, 0.0, 0.0);

  // Rotate the translational component of motion.
  Eigen::Vector3d trans_motion = rot_state*trans_motion_local;

  // Update the translational component of state. 
  state_.segment<3>(0) += trans_motion;

  // Get rotation component of motion as a pure quaternion.
  Eigen::Quaterniond rot_motion(0.0, motion(1)*dt, motion(2)*dt, motion(3)*dt);

  // Rotate the rotataionl component of state.
  Eigen::Quaterniond delta_rot_state = rot_state*rot_motion;

  rot_state.w() += 0.5*delta_rot_state.w();
  rot_state.x() += 0.5*delta_rot_state.x();
  rot_state.y() += 0.5*delta_rot_state.y();
  rot_state.z() += 0.5*delta_rot_state.z();

  rot_state.normalize();

  // Update the rotational component of state.
  state_(3) = rot_state.w();
  state_(4) = rot_state.x();
  state_(5) = rot_state.y();
  state_(6) = rot_state.z();

}

} // namespace fastslam

