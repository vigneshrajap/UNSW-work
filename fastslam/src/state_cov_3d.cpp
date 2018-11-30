
#include <fastslam/state_cov_3d.h>

namespace fastslam
{

StateCov3D::StateCov3D()
{

  cov_ = Matrix7d::Zero();
  cov_(0,0) = 0.0;
  cov_(1,1) = 0.0;
  cov_(2,2) = 0.0;
  cov_(3,3) = 10.0;
  cov_(4,4) = 0.0;
  cov_(5,5) = 0.0;
  cov_(6,6) = 10.0;

}

void StateCov3D::processModel(const Eigen::Vector4d & motion, double dt)
{

  State3D::processModel(motion, dt);

  double wx = motion(1);
  double wy = motion(2);
  double wz = motion(3);

  Matrix7d process_jacobian;
  process_jacobian << 1.0, 0.0, 0.0,  2.0*qw()*dt, 2.0*qx()*dt, -2.0*qy()*dt, -2.0*qz()*dt,
                      0.0, 1.0, 0.0,  2.0*qz()*dt, 2.0*qy()*dt,  2.0*qx()*dt,  2.0*qw()*dt,
                      0.0, 0.0, 1.0, -2.0*qy()*dt, 2.0*qz()*dt, -2.0*qw()*dt,  2.0*qx()*dt,
                      0.0, 0.0, 0.0,          1.0,  -0.5*wx*dt,   -0.5*wy*dt,   -0.5*wz*dt,
                      0.0, 0.0, 0.0,    0.5*wx*dt,         1.0,    0.5*wz*dt,   -0.5*wy*dt,
                      0.0, 0.0, 0.0,    0.5*wy*dt,  -0.5*wz*dt,          1.0,    0.5*wx*dt,
                      0.0, 0.0, 0.0,    0.5*wz*dt,   0.5*wy*dt,   -0.5*wy*dt,          1.0;

  Eigen::Matrix<double, 7, 4> motion_jacobian;
  motion_jacobian << qw()*qw() + qx()*qx() - qy()*qy() - qz()*qz(),       0.0,       0.0,      0.0,
                                      2.0*( qw()*qz() + qx()*qy()),       0.0,       0.0,      0.0,
                                      2.0*(-qw()*qy() + qx()*qz()),       0.0,       0.0,      0.0,
                                                               0.0, -0.5*qx(), -0.5*qy(), -0.5*qz(),
                                                               0.0,  0.5*qw(), -0.5*qz(),  0.5*qy(),
                                                               0.0,  0.5*qz(),  0.5*qw(), -0.5*qx(),
                                                               0.0, -0.5*qy(),  0.5*qx(),  0.5*qw();
  motion_jacobian *= dt;

  Eigen::Matrix4d motion_noise = Eigen::Matrix4d::Zero();
  motion_noise(0,0) = 0.001;
  motion_noise(1,1) = 0.001;
  motion_noise(2,2) = 0.001;
  motion_noise(3,3) = 0.001;

  cov_ = process_jacobian*cov_*process_jacobian.transpose() + motion_jacobian*motion_noise*motion_jacobian.transpose();

}

} // namespace fastslam

