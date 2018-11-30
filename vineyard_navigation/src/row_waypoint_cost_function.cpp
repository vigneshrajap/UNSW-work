
#include <vineyard_navigation/row_waypoint_cost_function.h>

PLUGINLIB_EXPORT_CLASS(vineyard_navigation::RowWaypointCostFunction, objective_planner::TrajectoryCostFunction);

namespace vineyard_navigation
{

RowWaypointCostFunction::RowWaypointCostFunction() : current_waypoint_idx_(1), loaded_(false), gps_origin_x_(0.0), gps_origin_y_(0.0), have_gps_origin_(false), use_gps_origin_(false)
{

}

bool RowWaypointCostFunction::loadParamsFromXml(XmlRpc::XmlRpcValue & xml_params)
{

  gps_origin_sub_ = node_handle_.subscribe("/gps_origin", 1, &RowWaypointCostFunction::gpsOriginCallback, this);
  waypoint_marker_pub_ = node_handle_.advertise<nav_msgs::Path>("/waypoint_marker", 1);

  ros::NodeHandle private_node_handle("~");
  std::string config_path;
  private_node_handle.getParam("vineyard_navigation_config_path", config_path);

  if (!xml_params.hasMember("row_waypoints_filename"))
  {
    ROS_ERROR("[vineyard_navigation::RowWaypointCostFunction]: missing param <row_waypoints_filename>");
    return false;
  }

  if (!xml_params.hasMember("row_order_filename"))
  {
    ROS_ERROR("[vineyard_navigation::RowWaypointCostFunction]: missing param <row_order_filename>");
    return false;
  }

  if (xml_params.hasMember("use_gps_origin"))
  {
    use_gps_origin_ = (bool)xml_params["use_gps_origin"];
  }

  if (use_gps_origin_)
  {
    getGpsOrigin();
  }

  std::string row_waypoints_filename = config_path + (std::string)xml_params["row_waypoints_filename"];
  std::string row_order_filename = config_path + (std::string)xml_params["row_order_filename"];

  std::string line;

  // Parse the row waypoints.
  std::ifstream row_waypoints_is(row_waypoints_filename.c_str());

  if (!row_waypoints_is.is_open())
  {
    ROS_ERROR("[vineyard_navigation::RowWaypointCostFunction]: unable to open file: %s", row_waypoints_filename.c_str());
    return false;
  }

  while (std::getline(row_waypoints_is, line))
  {

    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));

    std::vector<double> row_waypoint;
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(0)) - gps_origin_x_);
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(1)) - gps_origin_y_);
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(2)));
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(3)) - gps_origin_x_);
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(4)) - gps_origin_y_);
    row_waypoint.push_back(boost::lexical_cast<double>(tokens.at(5)));

    row_waypoints_.push_back(row_waypoint);

  }

  if (row_waypoints_.size() == 0)
  {
    ROS_ERROR("[vineyard_navigation::RowWaypointCostFunction]: no waypoints in file");
    return false;
  }

  // Parse the row order.
  std::ifstream row_order_is(row_order_filename.c_str());

  if (!row_order_is.is_open())
  {
    ROS_ERROR("[vineyard_navigation::RowWaypointCostFunction]: unable to open file: %s", row_order_filename.c_str());
    return false;
  }

  while (std::getline(row_order_is, line))
  {
    row_order_.push_back(boost::lexical_cast<int>(line));
  }

  return true;

}

void RowWaypointCostFunction::loadWaypointsWithRobotPosition()
{

  tf::StampedTransform transform;
  tf_listener_->lookupTransform("/map", "/base_link", ros::Time(), transform);
  double robot_x = transform.getOrigin().getX();
  double robot_y = transform.getOrigin().getY();

  // Find which end of the rows we are at using the first waypoint.
  double dx_0 = row_waypoints_.at(0).at(0) - robot_x;
  double dy_0 = row_waypoints_.at(0).at(1) - robot_y;
  double sq_dist_0 = dx_0*dx_0 + dy_0*dy_0;

  double dx_1 = row_waypoints_.at(0).at(3) - robot_x;
  double dy_1 = row_waypoints_.at(0).at(4) - robot_y;
  double sq_dist_1 = dx_1*dx_1 + dy_1*dy_1;

  int current_idx;
  if (sq_dist_0 < sq_dist_1)
  {
    current_idx = 0;
  }
  else
  {
    current_idx = 1;
  }

  parseRowWaypointsAndOrder(row_waypoints_, row_order_, current_idx);

  loaded_ = true;

}

void RowWaypointCostFunction::parseRowWaypointsAndOrder(const std::vector<std::vector<double> > & row_waypoints, const std::vector<int> & row_order, int current_idx)
{

  int current_row; 
  // Calculate the waypoints.
  // Initialise with the first waypoint.
  for (std::vector<int>::const_iterator row = row_order.begin(); row != row_order.end(); ++row)
  {

    std::vector<double> waypoint(2);

    // Initialise with the first waypoint.
    if (row == row_order.begin())
    {

      waypoint.at(0) = row_waypoints.at(*row).at(3*current_idx);
      waypoint.at(1) = row_waypoints.at(*row).at(3*current_idx + 1);
      waypoints_.push_back(waypoint);
      current_idx = (current_idx + 1) % 2;
      waypoint.at(0) = row_waypoints.at(*row).at(3*current_idx);
      waypoint.at(1) = row_waypoints.at(*row).at(3*current_idx + 1);
      waypoints_.push_back(waypoint);

    }
    else
    {

      // Add the end of each row as an intermediate waypoint.
      if (current_row < *row)
      {
        for (int i = current_row + 1; i <= *row; ++i)
        {
          waypoint.at(0) = row_waypoints.at(i).at(3*current_idx);
          waypoint.at(1) = row_waypoints.at(i).at(3*current_idx + 1);
          waypoints_.push_back(waypoint);
        }
      }
      else
      {
        for (int i = current_row - 1; i >= *row; --i)
        {
          waypoint.at(0) = row_waypoints.at(i).at(3*current_idx);
          waypoint.at(1) = row_waypoints.at(i).at(3*current_idx + 1);
          waypoints_.push_back(waypoint);

        }
      }

      // Move to the other end of the rows.
      current_idx = (current_idx + 1) % 2;
      waypoint.at(0) = row_waypoints.at(*row).at(3*current_idx);
      waypoint.at(1) = row_waypoints.at(*row).at(3*current_idx + 1);
      waypoints_.push_back(waypoint);

    }

    current_row = *row;

  }

}

bool RowWaypointCostFunction::initialise(boost::shared_ptr<tf::TransformListener> & tf_listener, boost::shared_ptr<costmap_2d::Costmap2DROS> & local_costmap_ros, boost::shared_ptr<costmap_2d::Costmap2DROS> & global_costmap_ros)
{
  tf_listener_ = tf_listener;
  return true;
}

bool RowWaypointCostFunction::prepare()
{

  if (!loaded_)
  {
    loadWaypointsWithRobotPosition();
  }

  static int count = 0;
  if (count++ % 100 == 0)
  {
    publishWaypoints();
  }

  return true;

}

double RowWaypointCostFunction::scoreTrajectory(objective_planner::Trajectory & traj)
{

  double score = 0.0;

  double x_0, y_0, th_0;
  traj.getPoint(0, x_0, y_0, th_0);
  checkNextWaypoint(x_0, y_0);

  double dist_along_line = 0.0;
  double dist_travelled = 0.0;
  double prev_x = 0.0, prev_y = 0.0;

  // Score based on the average distance from the trajectory to the line defining the centre of the row.
  for (int i = 0; i < traj.size(); ++i)
  {
    
    double x, y, th;
    traj.getPoint(i, x, y, th);

    if (i == 0)
    {
      prev_x = x;
      prev_y = y;
    }

    double dx = x - prev_x;
    double dy = y - prev_y;
    dist_travelled += sqrt(dx*dx + dy*dy); 

    Eigen::Vector2d line_pt(waypoints_.at(current_waypoint_idx_-1).at(0), waypoints_.at(current_waypoint_idx_-1).at(1));
    Eigen::Vector2d line_dir(waypoints_.at(current_waypoint_idx_).at(0) - waypoints_.at(current_waypoint_idx_-1).at(0),
                             waypoints_.at(current_waypoint_idx_).at(1) - waypoints_.at(current_waypoint_idx_-1).at(1));
    line_dir.normalize();

    Eigen::Vector2d traj_pt(x, y);

    double dist = ((line_pt - traj_pt) - ((line_pt - traj_pt).dot(line_dir))*line_dir).norm();
    score += dist;

    Eigen::Vector2d prev_traj_pt(prev_x, prev_y);
    dist_along_line += (traj_pt - prev_traj_pt).dot(line_dir);

    prev_x = x;
    prev_y = y;

  }

  return score + dist_travelled - dist_along_line;
 
}

void RowWaypointCostFunction::checkNextWaypoint(double x, double y)
{

  double dx = waypoints_.at(current_waypoint_idx_).at(0) - x;
  double dy = waypoints_.at(current_waypoint_idx_).at(1) - y;

  double sq_dist = dx*dx + dy*dy;

  if (sq_dist < 2.0*2.0)
  {
    ++current_waypoint_idx_;
  }

}

void RowWaypointCostFunction::getGpsOrigin()
{

  while (!have_gps_origin_)
  {
    ROS_INFO("[vineyard_navigation::RowWaypointCostFunction]: waiting for GPS origin");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

}

void RowWaypointCostFunction::gpsOriginCallback(const nav_msgs::Odometry::ConstPtr & gps_origin)
{

  gps_origin_x_ = gps_origin->pose.pose.position.x;
  gps_origin_y_ = gps_origin->pose.pose.position.y;

  have_gps_origin_ = true;

  ROS_INFO("[vineyard_navigation::RowWaypointCostFunction]: GPS origin set to (%f, %f)", gps_origin_x_, gps_origin_y_);

}

void RowWaypointCostFunction::publishWaypoints()
{

  nav_msgs::Path waypoint_marker;
  waypoint_marker.header.frame_id = "/odom";

  for (int i = 0; i < waypoints_.size(); ++i)
  {

    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "/odom";

    waypoint.pose.position.x = waypoints_.at(i).at(0);
    waypoint.pose.position.y = waypoints_.at(i).at(1);
    waypoint.pose.position.z = 0.0;

    waypoint_marker.poses.push_back(waypoint);

  }

  waypoint_marker_pub_.publish(waypoint_marker);

}

} // namespace vineyard_navigation

