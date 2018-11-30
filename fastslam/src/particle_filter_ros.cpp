
#include <fastslam/particle_filter_ros.h>

namespace fastslam
{

ParticleFilterROS::ParticleFilterROS() : linear_vel_(0.0), angular_vel_x_(0.0), angular_vel_y_(0.0), angular_vel_z_(0.0), have_gps_origin_(false)
{

  odom_sub_ = node_handle_.subscribe("/encoder", 1, &ParticleFilterROS::odomCallback, this);

  imu_sub_ = node_handle_.subscribe("/imu/data", 1, &ParticleFilterROS::imuCallback, this);

  gps_sub_ = node_handle_.subscribe("/gps", 20, &ParticleFilterROS::gpsCallback, this);

  setNParticles(50);

  initParticles();

}

void ParticleFilterROS::spin()
{


  while (ros::ok())
  {
    ros::spinOnce();
    broadcastTransforms();
    ros::Rate(200.0).sleep();
  }

}

void ParticleFilterROS::odomCallback(const nav_msgs::Odometry::ConstPtr & odom)
{

  if (prev_odom_imu_time_ == ros::Time())
  {
    prev_odom_imu_time_ = odom->header.stamp;
  }

  linear_vel_ = odom->twist.twist.linear.x;

  double dt = (odom->header.stamp - prev_odom_imu_time_).toSec();
  prev_odom_imu_time_ = odom->header.stamp;

  Eigen::Vector4d motion;
  motion << linear_vel_,
            angular_vel_x_,
            angular_vel_y_,
            angular_vel_z_;

  sample(motion, dt);

}

void ParticleFilterROS::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{

  if (prev_odom_imu_time_ == ros::Time())
  {
    prev_odom_imu_time_ = imu->header.stamp;
  }

  angular_vel_x_ = imu->angular_velocity.x;
  angular_vel_y_ = imu->angular_velocity.y;
  angular_vel_z_ = imu->angular_velocity.z;

  double dt = (imu->header.stamp - prev_odom_imu_time_).toSec();
  prev_odom_imu_time_ = imu->header.stamp;

  Eigen::Vector4d motion;
  motion << linear_vel_,
            angular_vel_x_,
            angular_vel_y_,
            angular_vel_z_;

  sample(motion, dt);

}

void ParticleFilterROS::gpsCallback(const nav_msgs::Odometry::ConstPtr & gps)
{

  if (!have_gps_origin_)
  {
    gps_origin_x_ = gps->pose.pose.position.x; 
    gps_origin_y_ = gps->pose.pose.position.y; 
    gps_origin_z_ = gps->pose.pose.position.z; 
    have_gps_origin_ = true;
  }

  Eigen::Vector3d meas_mean;
  meas_mean << gps->pose.pose.position.x - gps_origin_x_,
               gps->pose.pose.position.y - gps_origin_y_,
               gps->pose.pose.position.z - gps_origin_z_;

  Eigen::Matrix3d meas_cov = Eigen::Matrix3d::Zero();
  meas_cov(0,0) = gps->pose.covariance[ 0];
  meas_cov(1,1) = gps->pose.covariance[ 7];
  meas_cov(2,2) = gps->pose.covariance[14];

  resample(meas_mean, meas_cov);

}

void ParticleFilterROS::broadcastTransforms()
{

  for (int i = 0; i < n_particles_; ++i)
  {

    std::stringstream tf_frame_ss;
    tf_frame_ss << "/base_link_particle_" << i;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(particles_[i].tx(), particles_[i].ty(), particles_[i].tz()));
    transform.setRotation(tf::Quaternion(particles_[i].qx(), particles_[i].qy(), particles_[i].qz(), particles_[i].qw()));

    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, prev_odom_imu_time_, "/odom", tf_frame_ss.str()));

    if (i == 0)
    {
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, prev_odom_imu_time_, "/odom", "/base_link"));
    }

  }

}

} // namespace fastslam

