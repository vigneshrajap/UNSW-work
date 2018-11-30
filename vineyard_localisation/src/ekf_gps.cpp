
#include <vineyard_localisation/ekf_gps.h>

namespace vineyard_localisation
{

EKFGPS::EKFGPS() : have_gps_origin_(false), have_ground_augmentation_(false), init_from_gps_(false)
{

  ros::NodeHandle private_node_handle("~");
  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());

  std::string gps_topic("/gps");
  param_helper.getParamWithInfo("gps_topic", gps_topic);

  std::string fix_topic("/fix");
  param_helper.getParamWithInfo("fix_topic", fix_topic);

  std::string gps_origin_topic("/gps_origin");
  param_helper.getParamWithInfo("gps_origin_topic", gps_origin_topic);

  gps_sub_.subscribe(node_handle_, gps_topic, 20);
  fix_sub_ = node_handle_.subscribe(fix_topic, 20, &EKFGPS::fixCallback, this);
  gps_origin_pub_ = node_handle_.advertise<nav_msgs::Odometry>(gps_origin_topic, 1);

  param_helper.getParamWithInfo("init_from_gps", init_from_gps_);

}

void EKFGPS::init()
{

  gps_sequencer_.subscribe<0>(odom_sub_, boost::bind(&EKFOdometry::odomCallback, this, _1));
  gps_sequencer_.subscribe<1>(imu_sub_, boost::bind(&EKFAccelerometer::imuCallback, this, _1));
  gps_sequencer_.subscribe<2>(gps_sub_, boost::bind(&EKFGPS::gpsCallback, this, _1));

  gps_sequencer_.setUpdateRate(ros::Duration(update_rate_));
  gps_sequencer_.setDelay(ros::Duration(delay_));

  initOdometryCommon();
  initGpsCommon();

}

void EKFGPS::initGpsCommon()
{

  if (init_from_gps_)
  {
    while (!have_gps_origin_)
    {
      ROS_INFO("[vineyard_localisation::EKFGPS]: waiting for GPS reading");
      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }
  }

}

void EKFGPS::gpsCallback(const nav_msgs::Odometry::ConstPtr & gps)
{

  if (!have_gps_origin_)
  {

    initFromGps(*gps);

    gps_origin_x_ = gps->pose.pose.position.x;
    gps_origin_y_ = gps->pose.pose.position.y;
    gps_origin_z_ = gps->pose.pose.position.z;

    have_gps_origin_ = true;

  }

  static int count = 0;
  if (count++ % 100 == 0)
  {
    publishGpsOrigin();
  }

  Eigen::Vector3d gps_meas;
  gps_meas << gps->pose.pose.position.x - gps_origin_x_,
              gps->pose.pose.position.y - gps_origin_y_,
              gps->pose.pose.position.z - gps_origin_z_;

  Eigen::Vector3d gps_exp;
  gps_exp << tx(),
             ty(),
             tz();

  Eigen::Vector3d innovation = gps_meas - gps_exp;

  Eigen::Matrix<double, 3, 7> meas_jacobian_37;
  meas_jacobian_37 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXd meas_jacobian = Eigen::MatrixXd::Zero(3, size());
  meas_jacobian.block<3,7>(0,0) = meas_jacobian_37;

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Zero();
  meas_cov(0,0) = gps->pose.covariance[ 0];
  meas_cov(1,1) = gps->pose.covariance[ 7];
  meas_cov(2,2) = gps->pose.covariance[14];

  if (have_ground_augmentation_)
  {
    meas_cov /= 10.0;
  }

  Eigen::Matrix3d innovation_cov = meas_jacobian*cov_*meas_jacobian.transpose() + meas_cov;

  Eigen::MatrixXd kalman_gain = cov_*meas_jacobian.transpose()*innovation_cov.inverse();

  state_ += kalman_gain*innovation;

  normaliseQuat();

  cov_ = (Eigen::MatrixXd::Identity(size(), size()) - kalman_gain*meas_jacobian)*cov_;

  latest_time_ = gps->header.stamp;

}

void EKFGPS::fixCallback(const sensor_msgs::NavSatFix::ConstPtr & fix)
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

void EKFGPS::initFromGps(const nav_msgs::Odometry & gps)
{

  cov_(0,0) = gps.pose.covariance[ 0];
  cov_(1,1) = gps.pose.covariance[ 7];
  cov_(2,2) = gps.pose.covariance[14];

}

void EKFGPS::publishGpsOrigin()
{

  nav_msgs::Odometry gps_origin;
  gps_origin.pose.pose.position.x = gps_origin_x_;
  gps_origin.pose.pose.position.y = gps_origin_y_;
  gps_origin.pose.pose.position.z = gps_origin_z_;

  gps_origin_pub_.publish(gps_origin);

}

} // namespace vineyard_localisation

