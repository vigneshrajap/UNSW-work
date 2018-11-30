
#ifndef ODOMETRY_WITH_GPS_H_
#define ODOMETRY_WITH_GPS_H_

#include <fastslam/odometry_with_accelerometer.h>

#include <sensor_msgs/NavSatFix.h>

namespace fastslam
{

class OdometryWithGPS : public OdometryWithAccelerometer
{

public:

  OdometryWithGPS();

private:

  void gpsCallback(const nav_msgs::Odometry::ConstPtr & gps);

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr & fix);

  virtual Eigen::MatrixXd gpsJacobian();

  ros::Subscriber gps_sub_;
  
  ros::Subscriber fix_sub_;

  double gps_origin_x_;

  double gps_origin_y_;

  double gps_origin_z_;

  bool have_gps_origin_;

  bool have_ground_augmentation_;

};

} // namespace fastslam

#endif // #ifndef ODOMETRY_WITH_GPS_H_

