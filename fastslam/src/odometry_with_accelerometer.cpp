
#include <fastslam/odometry_with_accelerometer.h>

namespace fastslam
{

OdometryWithAccelerometer::OdometryWithAccelerometer()
{

}

void OdometryWithAccelerometer::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{

  OdometryOnly::imuCallback(imu);

  update(-imu->linear_acceleration.x, -imu->linear_acceleration.y, -imu->linear_acceleration.z);

}

void OdometryWithAccelerometer::update(double linear_acc_x, double linear_acc_y, double linear_acc_z)
{

  Eigen::Vector3d gravity_exp;
  gravity_exp << 2.0*(-state_.qw()*state_.qy() + state_.qx()*state_.qz()),
                 2.0*( state_.qw()*state_.qx() + state_.qy()*state_.qz()),
                 state_.qw()*state_.qw() - state_.qx()*state_.qx() - state_.qy()*state_.qy() + state_.qz()*state_.qz();
  gravity_exp *= GRAVITY;


  Eigen::Vector3d gravity_meas;
  gravity_meas << linear_acc_x,
                  linear_acc_y,
                  linear_acc_z;

  Eigen::Vector3d innovation = gravity_meas - gravity_exp;

  Eigen::Matrix<double, 3, 7> meas_jacobian;
  meas_jacobian << 0.0, 0.0, 0.0, -state_.qy(),  state_.qz(), -state_.qw(), state_.qx(),
                   0.0, 0.0, 0.0,  state_.qx(),  state_.qw(),  state_.qz(), state_.qy(),
                   0.0, 0.0, 0.0,  state_.qw(), -state_.qx(), -state_.qy(), state_.qz();

  meas_jacobian *= 2.0*GRAVITY;

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Zero();
  meas_cov(0,0) = 10.0;
  meas_cov(1,1) = 10.0;
  meas_cov(2,2) = 10.0;

  Eigen::Matrix3d innovation_cov = meas_jacobian*state_.cov()*meas_jacobian.transpose() + meas_cov;

  Eigen::Matrix<double, 7, 3> kalman_gain = state_.cov()*meas_jacobian.transpose()*innovation_cov.inverse();

  state_.state() += kalman_gain*innovation;

  state_.normaliseQuat();

  state_.cov() = (State3D::Matrix7d::Identity() - kalman_gain*meas_jacobian)*state_.cov();

}

void OdometryWithAccelerometer::transformEigen(Eigen::Vector3d & eigen_vec)
{

  geometry_msgs::Vector3Stamped ros_vec;
  ros_vec.header.frame_id = "/odom";
  ros_vec.header.stamp = ros::Time();
  ros_vec.vector.x = eigen_vec(0);
  ros_vec.vector.y = eigen_vec(1);
  ros_vec.vector.z = eigen_vec(2);
  
  geometry_msgs::Vector3Stamped transformed_ros_vec;

  if (tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(), ros::Duration(1.0)))
  {
    tf_listener_.transformVector("/base_link", ros_vec, transformed_ros_vec);

    eigen_vec(0) = transformed_ros_vec.vector.x;
    eigen_vec(1) = transformed_ros_vec.vector.y;
    eigen_vec(2) = transformed_ros_vec.vector.z;
  }

}

} // namespace fastslam

