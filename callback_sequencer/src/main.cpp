
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>

#include <callback_sequencer/callback_sequencer.h>

typedef sensor_msgs::Imu imu_t;
typedef sensor_msgs::LaserScan laser_t;

void laserScanCallback(const laser_t::ConstPtr & laser_scan)
{
  ROS_INFO("[LASER}: %d.%d", laser_scan->header.stamp.sec, laser_scan->header.stamp.nsec);
}

void imuCallback(const imu_t::ConstPtr & imu)
{
  ROS_INFO("[  IMU]: %d.%d", imu->header.stamp.sec, imu->header.stamp.nsec);
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "main");

  ros::NodeHandle nh;

  ros::Publisher laser_pub_0 = nh.advertise<laser_t>("laser_0", 1);
  ros::Publisher laser_pub_1 = nh.advertise<laser_t>("laser_1", 1);
  ros::Publisher laser_pub_2 = nh.advertise<laser_t>("laser_2", 1);
  ros::Publisher imu_pub_0 = nh.advertise<imu_t>("imu_0", 5);

  message_filters::Subscriber<laser_t> laser_sub_0(nh, "laser_0", 1);
  message_filters::Subscriber<laser_t> laser_sub_1(nh, "laser_1", 1);
  message_filters::Subscriber<laser_t> laser_sub_2(nh, "laser_2", 1);
  message_filters::Subscriber<imu_t> imu_sub_0(nh, "imu_0", 5);

  callback_sequencer::CallbackSequencer<laser_t, imu_t> cs(laser_sub_0, laserScanCallback, imu_sub_0, imuCallback);

  while (ros::ok())
  {

    for (int i = 0; i < 5; ++i)
    {
      imu_t imu;
      imu.header.stamp = ros::Time::now();
      imu_pub_0.publish(imu);
    }

    laser_t laser_scan;

    laser_scan.header.stamp = ros::Time::now() + ros::Duration(0.3);
    laser_pub_0.publish(laser_scan);

    laser_scan.header.stamp = ros::Time::now() + ros::Duration(0.2);
    laser_pub_1.publish(laser_scan);

    laser_scan.header.stamp = ros::Time::now() + ros::Duration(0.1);
    laser_pub_2.publish(laser_scan);

    ros::spinOnce();

    ros::Duration(0.05).sleep();

  }

}

