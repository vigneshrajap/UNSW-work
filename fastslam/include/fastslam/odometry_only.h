
#ifndef ODOMETRY_ONLY_H_
#define ODOMETRY_ONLY_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>

#include <fastslam/state_cov_3d.h>

namespace fastslam
{

class OdometryOnly
{

public:

  OdometryOnly();

  void spin();

protected:

  void odomCallback(const nav_msgs::Odometry::ConstPtr & odom);

  virtual void imuCallback(const sensor_msgs::Imu::ConstPtr & imu);

  void broadcastTransform();

  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;

  ros::Subscriber imu_sub_;

  ros::Time prev_time_;

  tf::TransformBroadcaster tf_broadcaster_;

  fastslam::StateCov3D state_;

  double linear_vel_;

  double angular_vel_x_;

  double angular_vel_y_;

  double angular_vel_z_;

  bool first_meas_;

};

} // namespace fastslam

#endif // #ifndef ODOMETRY_ONLY_H_

