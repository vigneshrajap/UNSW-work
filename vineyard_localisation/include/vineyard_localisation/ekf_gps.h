
#ifndef VITI_EKF_GPS_H_
#define VITI_EKF_GPS_H_

#include <sensor_msgs/NavSatFix.h>

#include <vineyard_localisation/ekf_accelerometer.h>

namespace vineyard_localisation
{

class EKFGPS : public EKFAccelerometer
{

public:

  EKFGPS();

  void init();

  void gpsCallback(const nav_msgs::Odometry::ConstPtr & gps);

protected:

  void initGpsCommon();

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr & fix);

  void gpsUpdate();

  message_filters::Subscriber<nav_msgs::Odometry> gps_sub_;

private:

  void initFromGps(const nav_msgs::Odometry & gps);

  void publishGpsOrigin();

  callback_sequencer::CallbackSequencer<nav_msgs::Odometry, sensor_msgs::Imu, nav_msgs::Odometry> gps_sequencer_;

  ros::Subscriber fix_sub_;

  ros::Publisher gps_origin_pub_;

  double gps_origin_x_;

  double gps_origin_y_;

  double gps_origin_z_;

  bool have_gps_origin_;

  bool have_ground_augmentation_;

  bool init_from_gps_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_GPS_H_

