
#include <fstream>

#include <ros/ros.h>

#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

#include <param_utils/param_utils.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "path_history");

  ros::NodeHandle private_nh("~");
  param_utils::ParamHelper param_helper(private_nh, ros::this_node::getName());

  // Parse params.
  std::string path_history_filepath("path_history.txt");
  param_helper.getParamWithInfo("path_history_filepath", path_history_filepath);

  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path_history", 1);

  tf::TransformListener tf;

  nav_msgs::Path path_history;
  path_history.header.frame_id = "/odom";

  std::ofstream path_history_os(path_history_filepath.c_str());
  path_history_os << std::fixed;

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
      continue;
    }
    catch (tf2::ExtrapolationException &)
    {
      continue;
    }

    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = "/odom";
    path_pose.pose.position.x = transform.getOrigin().getX();
    path_pose.pose.position.y = transform.getOrigin().getY();
    path_pose.pose.position.z = transform.getOrigin().getZ();

    path_history_os << path_pose.pose.position.x << ' ' << path_pose.pose.position.y << ' ' << path_pose.pose.position.z << ' ' << time_now.toSec() << std::endl;

    path_history.poses.push_back(path_pose);

    path_pub.publish(path_history);

    ros::Rate(10.0).sleep();

  }

  path_history_os.close();

  return 0;

}

