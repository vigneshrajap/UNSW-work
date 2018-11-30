
#include <fastslam/odometry_with_gps.h>

namespace fastslam
{

OdometryWithGPS::OdometryWithGPS() : have_gps_origin_(false), have_ground_augmentation_(false)
{

  gps_sub_ = nh_.subscribe("/gps", 20, &OdometryWithGPS::gpsCallback, this);

  fix_sub_ = nh_.subscribe("/fix", 20, &OdometryWithGPS::fixCallback, this);

}

void OdometryWithGPS::gpsCallback(const nav_msgs::Odometry::ConstPtr & gps)
{

  if (!have_gps_origin_)
  {
    gps_origin_x_ = gps->pose.pose.position.x;
    gps_origin_y_ = gps->pose.pose.position.y;
    gps_origin_z_ = gps->pose.pose.position.z;
    have_gps_origin_ = true;
  }

  Eigen::Vector3d gps_meas;
  gps_meas << gps->pose.pose.position.x - gps_origin_x_,
              gps->pose.pose.position.y - gps_origin_y_,
              gps->pose.pose.position.z - gps_origin_z_;

  Eigen::Vector3d gps_exp;
  gps_exp << state_.tx(),
             state_.ty(),
             state_.tz();

  Eigen::Vector3d innovation = gps_meas - gps_exp;

  Eigen::MatrixXd meas_jacobian = gpsJacobian();

  /*
  Eigen::Matrix<double, 3, 7> meas_jacobian;
  meas_jacobian << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  */

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Zero();
  meas_cov(0,0) = gps->pose.covariance[ 0];
  meas_cov(1,1) = gps->pose.covariance[ 7];
  meas_cov(2,2) = gps->pose.covariance[14];

  if (have_ground_augmentation_)
  {
    meas_cov /= 10.0;
  }

  Eigen::Matrix3d innovation_cov = meas_jacobian*state_.cov()*meas_jacobian.transpose() + meas_cov;

  Eigen::Matrix<double, 7, 3> kalman_gain = state_.cov()*meas_jacobian.transpose()*innovation_cov.inverse();

  state_.state() += kalman_gain*innovation;

  state_.normaliseQuat();

  state_.cov() = (State3D::Matrix7d::Identity() - kalman_gain*meas_jacobian)*state_.cov();

}

void OdometryWithGPS::fixCallback(const sensor_msgs::NavSatFix::ConstPtr & fix)
{

  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX)
  {
    have_ground_augmentation_ = true;
  }
  else
  {
    have_ground_augmentation_ = false;
  }

}

Eigen::MatrixXd OdometryWithGPS::gpsJacobian()
{

  Eigen::MatrixXd gps_jacobian = Eigen::MatrixXd::Zero(3,7);
  gps_jacobian(0,0) = 1.0;
  gps_jacobian(1,1) = 1.0;
  gps_jacobian(2,2) = 1.0;

  return gps_jacobian;

}

} // namespace fastslam

