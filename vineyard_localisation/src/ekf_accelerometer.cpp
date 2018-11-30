
#include <vineyard_localisation/ekf_accelerometer.h>

namespace vineyard_localisation
{

void EKFAccelerometer::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{
  EKFOdometry::imuCallback(imu);
  accelerometerUpdate(-imu->linear_acceleration.x, -imu->linear_acceleration.y, -imu->linear_acceleration.z);
}

void EKFAccelerometer::accelerometerUpdate(double linear_acc_x, double linear_acc_y, double linear_acc_z)
{

  Eigen::Vector3d gravity_exp;
  gravity_exp << 2.0*(-qw()*qy() + qx()*qz()),
                 2.0*( qw()*qx() + qy()*qz()),
                 qw()*qw() - qx()*qx() - qy()*qy() + qz()*qz();
  gravity_exp *= GRAVITY;


  Eigen::Vector3d gravity_meas;
  gravity_meas << linear_acc_x,
                  linear_acc_y,
                  linear_acc_z;

  Eigen::Vector3d innovation = gravity_meas - gravity_exp;

  // XXX Best initialiser?
  Eigen::Matrix<double, 3, 7> meas_jacobian_37;
  meas_jacobian_37 << 0.0, 0.0, 0.0, -qy(),  qz(), -qw(), qx(),
                      0.0, 0.0, 0.0,  qx(),  qw(),  qz(), qy(),
                      0.0, 0.0, 0.0,  qw(), -qx(), -qy(), qz();

  meas_jacobian_37 *= 2.0*GRAVITY;

  Eigen::MatrixXd meas_jacobian = Eigen::MatrixXd::Zero(3, size());
  meas_jacobian.block<3,7>(0,0) = meas_jacobian_37;

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Zero();
  meas_cov(0,0) = 100.0; // XXX 20
  meas_cov(1,1) = 100.0;
  meas_cov(2,2) = 100.0;

  Eigen::Matrix3d innovation_cov = meas_jacobian*cov_*meas_jacobian.transpose() + meas_cov;

  Eigen::MatrixXd kalman_gain = cov_*meas_jacobian.transpose()*innovation_cov.inverse();

  state_ += kalman_gain*innovation;

  normaliseQuat();

  cov_ = (Eigen::MatrixXd::Identity(size(), size()) - kalman_gain*meas_jacobian)*cov_;

}

} // namespace vineyard_localisation

