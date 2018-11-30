
#ifndef VITI_EKF_ACCELEROMETER_H_
#define VITI_EKF_ACCELEROMETER_H_

#include <vineyard_localisation/ekf_odometry.h>

#define GRAVITY -9.81

namespace vineyard_localisation
{

class EKFAccelerometer : public EKFOdometry
{

public:

  void imuCallback(const sensor_msgs::Imu::ConstPtr & imu); 

protected:

  void accelerometerUpdate(double linear_acc_x, double linear_acc_y, double linear_acc_z);

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_ACCELEROMETER_H_

