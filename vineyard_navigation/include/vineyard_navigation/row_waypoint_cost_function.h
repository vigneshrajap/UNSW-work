
#ifndef ROW_WAYPOINT_COST_FUNCTION_H_
#define ROW_WAYPOINT_COST_FUNCTION_H_

#include <fstream>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pluginlib/class_list_macros.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Dense>

#include <objective_planner/trajectory_cost_function.h>

#include <vineyard_row_detection/line_utils.h>

namespace vineyard_navigation
{

// XXX Rename to RowWaypoints.
class RowWaypointCostFunction : public objective_planner::TrajectoryCostFunction
{

public:

  RowWaypointCostFunction();

  std::vector<std::vector<double> > getWaypoints() { return waypoints_; }

  bool loadParamsFromXml(XmlRpc::XmlRpcValue & xml_params);

  void parseRowWaypointsAndOrder(const std::vector<std::vector<double> > & row_waypoints, const std::vector<int> & row_order, int current_idx);

  bool initialise(boost::shared_ptr<tf::TransformListener> & tf_listener, boost::shared_ptr<costmap_2d::Costmap2DROS> & local_costmap_ros, boost::shared_ptr<costmap_2d::Costmap2DROS> & global_costmap_ros);

  bool prepare();

  void loadWaypointsWithRobotPosition();

  double scoreTrajectory(objective_planner::Trajectory & traj);

  void checkNextWaypoint(double x, double y);

private:

  void getGpsOrigin();

  void gpsOriginCallback(const nav_msgs::Odometry::ConstPtr & gps_origin);

  void publishWaypoints();

  ros::NodeHandle node_handle_;

  ros::Subscriber gps_origin_sub_;

  ros::Publisher waypoint_marker_pub_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  std::vector<std::vector<double> > waypoints_;

  std::vector<std::vector<double> > row_waypoints_;

  std::vector<int> row_order_;

  int current_waypoint_idx_;

  bool loaded_;

  double gps_origin_x_;

  double gps_origin_y_;

  bool have_gps_origin_;

  bool use_gps_origin_;

};

} // namespace vineyard_navigation

#endif // #ifndef ROW_WAYPOINT_COST_FUNCTION_H_

