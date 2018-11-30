
#include <vineyard_navigation/row_following_cost_function.h>

PLUGINLIB_EXPORT_CLASS(vineyard_navigation::RowFollowingCostFunction, objective_planner::TrajectoryCostFunction);

namespace vineyard_navigation
{

RowFollowingCostFunction::RowFollowingCostFunction() : have_row_(false)
{

}

bool RowFollowingCostFunction::initialise(boost::shared_ptr<tf::TransformListener> & tf_listener, boost::shared_ptr<costmap_2d::Costmap2DROS> & local_costmap_ros, boost::shared_ptr<costmap_2d::Costmap2DROS> & global_costmap_ros)
{

  tf_listener_ = tf_listener;

  row_centre_sub_ = node_handle_.subscribe("row_centre", 1, &RowFollowingCostFunction::rowCentreCallback, this);

  // XXX Compile all with -Wall -Werror.
  return true;

}

bool RowFollowingCostFunction::prepare()
{
  ros::spinOnce();
  return true;
}

double RowFollowingCostFunction::scoreTrajectory(objective_planner::Trajectory & traj)
{

  // Score all trajectories equally if we haven't got an estimate of the row yet.
  if (!have_row_) { return 0.0; }

  double score = 0.0;

  // Score based on the average distance from the trajectory to the line defining the centre of the row.
  for (int i = 0; i < traj.size(); ++i)
  {
    
    double x, y, th;
    traj.getPoint(i, x, y, th);

    Eigen::Vector2f local_polar = vineyard_row_detection::shiftPolarOrigin(Eigen::Vector2f((float)row_radius_, (float)row_angle_), Eigen::Vector2f((float)x, (float)y));

    score += fabs(local_polar(0));

  }

  return score/(float)traj.size();

}

void RowFollowingCostFunction::rowCentreCallback(const geometry_msgs::Point::ConstPtr & row_centre)
{

  // The line defining the centre of the row is in polar coordinates.
  row_radius_ = row_centre->x;
  row_angle_  = row_centre->z;

//  ROS_INFO("Here!");

  have_row_ = true;

}

} // namespace vineyard_navigation

