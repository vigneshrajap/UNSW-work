
#ifndef VITI_EKF_ODOMETRY_H_
#define VITI_EKF_ODOMETRY_H_

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>

#include <vineyard_localisation/ekf_base.h>

#include <callback_sequencer/callback_sequencer.h>

namespace vineyard_localisation
{

class EKFOdometry : public EKFBase
{

public:

  EKFOdometry();

  void init();

  void spin();

  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr & odom);

protected:

  void initOdometryCommon();

  virtual void imuCallback(const sensor_msgs::Imu::ConstPtr & imu);

  void processModel(const Eigen::Vector4d & motion, double dt);

  callback_sequencer::CallbackSequencer<nav_msgs::Odometry, sensor_msgs::Imu> odometry_sequencer_;

  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;

  double linear_vel_;

  double angular_vel_x_;

  double angular_vel_y_;

  double angular_vel_z_;

  bool init_orientation_from_imu_;

  double initial_yaw_;

  ros::Time prev_time_;

  double update_rate_;
  
  double delay_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_ODOMETRY_H_

