
#include <ros/ros.h>

#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "path_history");

  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_history", 1);

  tf::TransformListener tf;

  nav_msgs::Path path_history;
  path_history.header.frame_id = "/odom";

  while (ros::ok())
  {
    
    tf::StampedTransform transform;
    ros::Time time_now = ros::Time::now();

    try
    {
      tf.waitForTransform("/odom", "/base_link", time_now, ros::Duration(1.0));
      tf.lookupTransform("/odom", "/base_link", time_now, transform);
    }
    catch (tf::LookupException &)
    {

    }
    catch (tf2::ExtrapolationException &)
    {

    }

    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = "/odom";
    path_pose.pose.position.x = transform.getOrigin().getX();
    path_pose.pose.position.y = transform.getOrigin().getY();
    path_pose.pose.position.z = transform.getOrigin().getZ();

    path_history.poses.push_back(path_pose);

    path_pub.publish(path_history);

    ros::Rate(10.0).sleep();

  }

  return 0;

}

