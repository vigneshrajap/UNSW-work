
#ifndef ROW_FOLLOWING_COST_FUNCTION_H_
#define ROW_FOLLOWING_COST_FUNCTION_H_

#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

#include <objective_planner/trajectory_cost_function.h>

#include <vineyard_row_detection/line_utils.h>

namespace vineyard_navigation
{

class RowFollowingCostFunction : public objective_planner::TrajectoryCostFunction
{

public:

  RowFollowingCostFunction();

  bool loadParamsFromXml(XmlRpc::XmlRpcValue & xml_params) { return true; }

  bool initialise(boost::shared_ptr<tf::TransformListener> & tf_listener, boost::shared_ptr<costmap_2d::Costmap2DROS> & local_costmap_ros, boost::shared_ptr<costmap_2d::Costmap2DROS> & global_costmap_ros);

  bool prepare(); 

  double scoreTrajectory(objective_planner::Trajectory & traj);

private:

  void rowCentreCallback(const geometry_msgs::Point::ConstPtr & row_centre);

  ros::NodeHandle node_handle_;

  ros::Subscriber row_centre_sub_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  double row_radius_;
 
  double row_angle_;

  bool have_row_;

};

} // namespace vineyard_navigation

#endif // #ifndef ROW_FOLLOWING_COST_FUNCTION_H_

