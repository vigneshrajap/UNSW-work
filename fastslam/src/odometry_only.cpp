
#include <fastslam/odometry_only.h>

namespace fastslam
{

OdometryOnly::OdometryOnly() : linear_vel_(0.0), angular_vel_x_(0.0), angular_vel_y_(0.0), angular_vel_z_(0.0), first_meas_(false)
{

  odom_sub_ = nh_.subscribe("/encoder", 10, &OdometryOnly::odomCallback, this);

  imu_sub_ = nh_.subscribe("/imu/data", 200, &OdometryOnly::imuCallback, this);

  prev_time_ = ros::Time(0, 0);

}

void OdometryOnly::spin()
{

  while (ros::ok())
  {

    broadcastTransform();

    ros::spinOnce();

    ros::Rate(200.0).sleep();

  }

}

void OdometryOnly::odomCallback(const nav_msgs::Odometry::ConstPtr & odom)
{

  if (!first_meas_)
  {
    prev_time_ = odom->header.stamp;
    first_meas_ = true;
  }

  linear_vel_ = odom->twist.twist.linear.x;

  ros::Time current_time = odom->header.stamp;
  double dt = (current_time - prev_time_).toSec();
  prev_time_ = current_time;

  Eigen::Vector4d motion;
  motion << linear_vel_,
            angular_vel_x_,
            angular_vel_y_,
            angular_vel_z_;

  state_.processModel(motion, dt);

}

void OdometryOnly::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{

  if (!first_meas_)
  {
    prev_time_ = imu->header.stamp;
    first_meas_ = true;
  }

  angular_vel_x_ = imu->angular_velocity.x;
  angular_vel_y_ = imu->angular_velocity.y;
  angular_vel_z_ = imu->angular_velocity.z;

  ros::Time current_time = imu->header.stamp;
  double dt = (current_time - prev_time_).toSec();
  prev_time_ = current_time;

  Eigen::Vector4d motion;
  motion << linear_vel_,
            angular_vel_x_,
            angular_vel_y_,
            angular_vel_z_;

  state_.processModel(motion, dt);

}

void OdometryOnly::broadcastTransform()
{

  if (first_meas_)
  {

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state_.tx(), state_.ty(), state_.tz()));
    transform.setRotation(tf::Quaternion(state_.qx(), state_.qy(), state_.qz(), state_.qw()));

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, prev_time_, "/odom", "/base_link"));

  }

}

} // namespace fastslam

