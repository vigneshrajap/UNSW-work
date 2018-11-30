
#include <vineyard_localisation/ekf_lines.h>

namespace vineyard_localisation
{

EKFLines::EKFLines() : radius_weight_(0.5), angle_weight_(0.5), line_match_threshold_(1.0), candidate_start_value_(2), candidate_remove_value_(0), candidate_add_value_(4)
{

  // Set up publishers and subscribers.
  ros::NodeHandle private_nh("~");
  param_utils::ParamHelper param_helper(private_nh, ros::this_node::getName());

  std::string lines_topic("lines");
  param_helper.getParamWithInfo("lines_topic", lines_topic);

  lines_sub_.subscribe(node_handle_, lines_topic, 1);

  std::string state_lines_topic("/state_lines");
  param_helper.getParamWithInfo("state_lines_topic", state_lines_topic);

  state_lines_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>(state_lines_topic, 1);

  markers_pub_ = node_handle_.advertise<visualization_msgs::Marker>("/state_lines_marker", 1);

  // Parse radius and angle weights used when calculating the distance between lines.
  param_helper.getParamWithInfo("radius_weight", radius_weight_);
  param_helper.getParamWithInfo("angle_weight" , angle_weight_);

  // Parse the threshold for 2 lines to be considered matched.
  param_helper.getParamWithInfo("line_match_threshold", line_match_threshold_);

  // Parse the candidate values.
  param_helper.getParamWithInfo("candidate_start_value" , candidate_start_value_);
  param_helper.getParamWithInfo("candidate_remove_value", candidate_remove_value_);
  param_helper.getParamWithInfo("candidate_add_value"   , candidate_add_value_);

  // Set up the timer for detecting the endpoints of the rows.
  point_cloud_assemble_timer_ = node_handle_.createTimer(ros::Duration(0.5), &EKFLines::assemblePointCloud, this);

}

void EKFLines::init()
{

  line_sequencer_.subscribe<0>(odom_sub_, boost::bind(&EKFOdometry::odomCallback, this, _1));
  line_sequencer_.subscribe<1>(imu_sub_, boost::bind(&EKFAccelerometer::imuCallback, this, _1));
  line_sequencer_.subscribe<2>(gps_sub_, boost::bind(&EKFGPS::gpsCallback, this, _1));
  line_sequencer_.subscribe<3>(lines_sub_, boost::bind(&EKFLines::linesCallback, this, _1));

  line_sequencer_.setUpdateRate(ros::Duration(update_rate_));
  line_sequencer_.setDelay(ros::Duration(delay_));

  initOdometryCommon();
  initGpsCommon();

}

void EKFLines::linesCallback(const geometry_msgs::PoseArray::ConstPtr & lines)
{

  ROS_INFO("Lines detected!");

  line_vec_t extracted_lines;
  
  line_endpoints_.clear();

  // Parse the detected lines.
  // The lines alternate between start and end points.
  for (std::vector<geometry_msgs::Pose>::const_iterator pose = lines->poses.begin(); pose != lines->poses.end(); pose += 2)
  {

    Eigen::Matrix<double,6,1> line_coefficients;
    line_coefficients(0) = pose->position.x;
    line_coefficients(1) = pose->position.y;
    line_coefficients(2) = pose->position.z;
    line_coefficients(3) = (pose+1)->position.x - pose->position.x;
    line_coefficients(4) = (pose+1)->position.y - pose->position.y;
    line_coefficients(5) = (pose+1)->position.z - pose->position.z;

    extracted_lines.push_back(line_t(line_coefficients));
//    extracted_lines.back().makeRadiusPositive();

    line_t local_line(extracted_lines.back());
    local_line.shiftOrigin(tx(), ty());

    line_endpoints_.push_back(Eigen::Vector2d(pose->position.x, pose->position.y));
    line_endpoints_.push_back(Eigen::Vector2d((pose+1)->position.x, (pose+1)->position.y));

  }

  std::vector<std::pair<int, int> > state_indices, candidate_indices;
  std::vector<int> unmatched_indices;

  // Match the detected lines to existing lines.
  matchLines(extracted_lines, state_indices, candidate_indices, unmatched_indices);

  /*
  ROS_INFO("[vineyard_localisation::EKFLines]: %zu lines extracted", extracted_lines.size());
  ROS_INFO("[vineyard_localisation::EKFLines]: %zu lines matched to state lines", state_indices.size());
  ROS_INFO("[vineyard_localisation::EKFLines]: %zu lines matched to candidate lines", candidate_indices.size());
  ROS_INFO("[vineyard_localisation::EKFLines]: %zu lines unmatched", unmatched_indices.size());
  */

  // Handle matches to lines that are in the state.
  handleStateLines(extracted_lines, state_indices);

  // Handle matches to lines that are candidates from state lines.
  handleCandidateLines(extracted_lines, candidate_indices);

  // Handle line that do not match to either state or candidate lines.
  handleUnmatchedLines(extracted_lines, unmatched_indices);

  // Publish the lines in the state vector.
//  publishStateLines(*lines, state_indices);

  // Update the latest time that the EKF is accurate at.
  latest_time_ = lines->header.stamp;

  /*
  line_vec_t state_lines = linesFromState();
  for (line_vec_it_t state_line = state_lines.begin(); state_line != state_lines.end(); ++state_line)
  {
   
    // Convert state line to local coordinates.
    line_t local_state_line(*state_line);
    local_state_line.shiftOrigin(tx(), ty());

    ROS_INFO("State line radius: %f angle: %f", local_state_line.radius(), local_state_line.angle());

  }
  */

}

void EKFLines::matchLines(const line_vec_t & lines, std::vector<std::pair<int, int> > & state_indices, std::vector<std::pair<int, int> > & candidate_indices, std::vector<int> & unmatched_indices)
{

  int line_idx = 0;
  for (line_vec_const_it_t line = lines.begin(); line != lines.end(); ++line, ++line_idx)
  {

    // Convert line to local coordinates.
    line_t local_line(*line);
    local_line.shiftOrigin(tx(), ty());
    local_line.makeRadiusPositive();

    // Compare against state lines.
    // Get the current lines from the state vector.
    line_vec_t state_lines = linesFromState();

    double min_dist = std::numeric_limits<double>::max();
    int min_idx;
    int state_idx = 0;
    for (line_vec_const_it_t state_line = state_lines.begin(); state_line != state_lines.end(); ++state_line, ++state_idx)
    {
      
      // Convert line to local coordinates.
      line_t local_state_line(*state_line);
      local_state_line.shiftOrigin(tx(), ty());
      local_state_line.makeRadiusPositive();

      double dist = lineToLineDistance(local_line, local_state_line);
      
      if (dist < min_dist)
      {
        min_idx = state_idx;
        min_dist = dist;
      }
    }

    if (min_dist < line_match_threshold_)
    {
      state_indices.push_back(std::pair<int, int>(line_idx, min_idx));
      continue;
    }

    // Compare against candidate lines.
    // XXX separate variables for state and candidate.
    min_dist = std::numeric_limits<double>::max();
    int candidate_idx = 0;
    for (line_vec_const_it_t candidate_line = candidate_lines_.begin(); candidate_line != candidate_lines_.end(); ++candidate_line, ++candidate_idx)
    {

      // Convert line to local coordinates.
      line_t local_candidate_line(*candidate_line);
      local_candidate_line.shiftOrigin(tx(), ty());
      local_candidate_line.makeRadiusPositive();

      double dist = lineToLineDistance(local_line, local_candidate_line);
      
      if (dist < min_dist)
      {
        min_idx = candidate_idx;
        min_dist = dist;
      }

    }

    if (min_dist < line_match_threshold_)
    {
      candidate_indices.push_back(std::pair<int, int>(line_idx, min_idx));
      continue;
    }

    // Line is unmatched.
    unmatched_indices.push_back(line_idx);

  }

}

void EKFLines::handleStateLines(line_vec_t & lines, const std::vector<std::pair<int, int> > & indices)
{

  line_vec_t state_lines = linesFromState();

  for (std::vector<std::pair<int, int> >::const_iterator idx_pair = indices.begin(); idx_pair != indices.end(); ++idx_pair)
  {

    // Convert matched line to local coordinates.
    line_t local_line(lines.at(idx_pair->first));
    local_line.makeRadiusPositive();
    local_line.shiftOrigin(tx(), ty());
   
    // Convert state line to local coordinates.
    line_t local_state_line(state_lines.at(idx_pair->second));
    local_state_line.makeRadiusPositive();
    local_state_line.shiftOrigin(tx(), ty());

    // Perform the EKF update.
    ekfUpdate(local_line, local_state_line, 7 + 2*idx_pair->second, lines);

  }

}

void EKFLines::ekfUpdate(line_t & observed_line, line_t & expected_line, int state_line_idx, line_vec_t & lines)
{

  //observed_line.makeRadiusPositive();
  //expected_line.makeRadiusPositive();

  double delta_radius = observed_line.radius() - expected_line.radius();
  double delta_angle = observed_line.angle() - expected_line.angle();

  while (delta_angle >= M_PI) { delta_angle -= 2.0*M_PI; }
  while (delta_angle < -M_PI) { delta_angle += 2.0*M_PI; }

  Eigen::Vector2d innovation(delta_radius, delta_angle); 

  Eigen::MatrixXd meas_jacobian = Eigen::MatrixXd::Zero(2, size());

  double robot_theta = 2.0*acos(qw()/sqrt(qw()*qw() + qz()*qz()))*(qz() > 0 ? 1.0 : -1.0);
  double line_theta = observed_line.angle();// - robot_theta;

  meas_jacobian(0,0) = -cos(line_theta);
  meas_jacobian(0,1) = -sin(line_theta);
  meas_jacobian(0, state_line_idx) = 1.0;
  meas_jacobian(0, state_line_idx + 1) = tx()*sin(line_theta) - ty()*cos(line_theta);
  meas_jacobian(1,3) =  2.0*fabs(qw())/sqrt(qw()*qw() + qz()*qz());
  meas_jacobian(1,6) = -2.0*fabs(qz())/sqrt(qw()*qw() + qz()*qz());
  meas_jacobian(1, state_line_idx + 1) = 1.0;

  /*
  Eigen::Matrix2d meas_cov = Eigen::Matrix2d::Zero();
  meas_cov(0,0) = 0.5*0.5;
  meas_cov(1,1) = (10.0*M_PI/180.0)*(10.0*M_PI/180.0);
  */
  Eigen::Matrix2d meas_cov = lineCovariance(observed_line, state_line_idx);

  Eigen::Matrix2d innovation_cov = meas_jacobian*cov_*meas_jacobian.transpose() + meas_cov;

  Eigen::MatrixXd kalman_gain = cov_*meas_jacobian.transpose()*innovation_cov.inverse();

  Eigen::VectorXd delta_state = kalman_gain*innovation;

  state_ += delta_state;
  normaliseQuat();

  cov_ = (Eigen::MatrixXd::Identity(size(), size()) - kalman_gain*meas_jacobian)*cov_;

  // Apply the update to the other observed lines.
  /*
  for (line_vec_it_t line = lines.begin(); line != lines.end(); ++line)
  {
    line->shiftOrigin(delta_state(0), delta_state(1));
    line->angle() += delta_state(state_line_idx + 1);
  }
  */

}

Eigen::MatrixXd EKFLines::lineCovariance(const line_t & line, int line_idx)
{

  Eigen::MatrixXd line_jacobian = Eigen::MatrixXd::Zero(2, size());

  line_jacobian(0,0) = -cos(line.angle());
  line_jacobian(0,1) = -sin(line.angle());
  line_jacobian(0, line_idx) = 1.0;
  line_jacobian(0, line_idx + 1) = tx()*sin(line.angle()) - ty()*cos(line.angle());
  line_jacobian(1,3) =  2.0*fabs(qw())/sqrt(qw()*qw() + qz()*qz());
  line_jacobian(1,6) = -2.0*fabs(qz())/sqrt(qw()*qw() + qz()*qz());
  line_jacobian(1, line_idx + 1) = 1.0;

  return line_jacobian*cov_*line_jacobian.transpose() + newLineCovariance();

}

void EKFLines::handleCandidateLines(const line_vec_t & lines, const std::vector<std::pair<int, int> > & indices)
{

  // Decrement all counts.
  for (std::vector<int>::iterator count = candidate_line_counts_.begin(); count != candidate_line_counts_.end(); ++count)
  {
    --(*count);
  }

  // Increment all counts that have been matched and replace the candidate with the newly observed line.
  for (std::vector<std::pair<int, int> >::const_iterator idx_pair = indices.begin(); idx_pair != indices.end(); ++idx_pair)
  {
    candidate_lines_.at(idx_pair->second) = lines.at(idx_pair->first);
    // Add 2 to cancel out the decrement above.
    candidate_line_counts_.at(idx_pair->second) += 2;
  } 

  for (int i = candidate_lines_.size() - 1; i >= 0; --i)
  {

    // Check if the line needs to be added to the state or is no longer a valid candidate.
    bool changed = false;
    if (candidate_line_counts_.at(i) <= candidate_remove_value_)
    {
      changed = true;
      ROS_INFO("[vineyard_localisation::EKFLines]: removed candidate line");
    }
    else if(candidate_line_counts_.at(i) >= candidate_add_value_)
    {
      addLineToState(candidate_lines_.at(i));
      changed = true;
      ROS_INFO("[vineyard_localisation::EKFLines]: added candidate line to state");
    }

    // If the line has changed state, remove it from the list of candidates.
    if (changed)
    {
      candidate_lines_.erase(candidate_lines_.begin() + i);
      candidate_line_counts_.erase(candidate_line_counts_.begin() + i);
    }

  }

}

void EKFLines::addLineToState(line_t & new_state_line)
{

  new_state_line.makeRadiusPositive();

  state_.conservativeResize(state_.rows() + 2);
  state_.segment<2>(state_.rows() - 2) = Eigen::Vector2d(new_state_line.radius(), new_state_line.angle());

  Eigen::MatrixXd old_cov = cov_;
  cov_ = Eigen::MatrixXd::Zero(state_.rows(), state_.rows());
  cov_.block(0, 0, old_cov.rows(), old_cov.cols()) = old_cov;
  cov_.block(old_cov.rows(), old_cov.cols(), 2, 2) = newLineCovariance();
  cov_.block(old_cov.rows(), old_cov.cols(), 2, 2) = lineCovariance(new_state_line, state_.rows() - 2);

  state_line_min_proj_.push_back( std::numeric_limits<double>::max());
  state_line_max_proj_.push_back(-std::numeric_limits<double>::max());

}

Eigen::MatrixXd EKFLines::newLineCovariance()
{

  Eigen::MatrixXd new_line_cov = Eigen::MatrixXd::Zero(2, 2);
  new_line_cov(0,0) = 0.10*0.10;
  new_line_cov(1,1) = (5.0*M_PI/180.0)*(5.0*M_PI/180.0);

  return new_line_cov;

}

void EKFLines::handleUnmatchedLines(const line_vec_t & lines, const std::vector<int> & indices)
{

  // Add all of the unmatched lines to the list of candidate lines.
  for (std::vector<int>::const_iterator idx = indices.begin(); idx != indices.end(); ++idx)
  {
    candidate_lines_.push_back(lines.at(*idx));
    candidate_line_counts_.push_back(candidate_start_value_);
    ROS_INFO("[vineyard_localisation::EKFLines]: new candidate line");
  }

}

/*
void EKFLines::publishStateLines(const geometry_msgs::PoseArray & lines, const std::vector<std::pair<int, int> > & state_indices)
{

  geometry_msgs::PoseArray state_lines;
  state_lines.header = lines.header;

  for (std::vector<std::pair<int, int> >::const_iterator idx_pair = state_indices.begin(); idx_pair != state_indices.end(); ++idx_pair)
  {
    state_lines.poses.push_back(lines.poses.at(idx_pair->first));    
  }

  state_lines_pub_.publish(state_lines);

}
*/

double EKFLines::lineToLineDistance(const line_t & line_1, const line_t & line_2)
{

  double delta_radius = line_1.radius() - line_2.radius();
  double delta_angle  = line_1.angle()  - line_2.angle();

  while (delta_angle >= M_PI) { delta_angle -= 2.0*M_PI; }
  while (delta_angle < -M_PI) { delta_angle += 2.0*M_PI; }

  return sqrt(radius_weight_*delta_radius*delta_radius + angle_weight_*delta_angle*delta_angle);

}

line_vec_t EKFLines::linesFromState()
{

  line_vec_t state_lines;

  for (int i = 7; i < state_.rows(); i += 2)
  {
    state_lines.push_back((Eigen::Vector2d)state_.segment<2>(i));
  }

  return state_lines;

}

void EKFLines::assemblePointCloud(const ros::TimerEvent &)
{

    pcl::PointCloud<pcl::PointXYZ> cloud;
    ros::Time assemble_time;
    point_cloud_assembler_.assemblePCL(cloud, ros::Time::now() - ros::Duration(5.0), ros::Time::now(), assemble_time);

    findRowEndpoints(cloud);

}

void EKFLines::findRowEndpoints(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{

  line_vec_t state_lines = linesFromState();

  // Don't do anything if we don't have any state lines yet.
  if (state_lines.size() == 0)
  {
    return;
  }

  std::vector<Eigen::Vector2d> line_points(state_lines.size());
  std::vector<Eigen::Vector2d> line_directions(state_lines.size());

  // Parametrise the lines in point/direction form.
  for (int i = 0; i < state_lines.size(); ++i)
  {
    double radius = state_lines.at(i).radius();
    double angle  = state_lines.at(i).angle();
    line_points.at(i) = Eigen::Vector2d(radius*cos(angle), radius*sin(angle));
    // XXX
    line_directions.at(i) = Eigen::Vector2d(radius*cos(angle + M_PI_2), radius*sin(angle + M_PI_2));
    line_directions.at(i).normalize();
  }

  /*
  std::vector<double> min_proj(state_lines.size());
  std::fill(min_proj.begin(), min_proj.end(), std::numeric_limits<double>::max());
  std::vector<double> max_proj(state_lines.size());
  std::fill(max_proj.begin(), max_proj.end(), -std::numeric_limits<double>::max());
  */

  // XXX not complete accurate, as it is only updating using the latest observed line endpoints.

  for (std::vector<Eigen::Vector2d>::iterator line_endpoint = line_endpoints_.begin(); line_endpoint != line_endpoints_.end(); ++line_endpoint)
  {

    // Find the line that this point is closest to.
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;
    for (int i = 0; i < line_points.size(); ++i)
    {
      Eigen::Vector2d delta_point = line_points.at(i) - *line_endpoint;
      double dist = (delta_point - (delta_point.dot(line_directions.at(i)))*line_directions.at(i)).norm();
      if (dist < min_dist)
      {
        min_idx = i;
        min_dist = dist;
      }
    }

    // Find the points with the largest and smallest projections for each line.
    // These will define the endpoints of each line.
    double proj = (*line_endpoint - line_points.at(min_idx)).dot(line_directions.at(min_idx));

    if (proj < state_line_min_proj_.at(min_idx))
    {
      state_line_min_proj_.at(min_idx) = proj;
    }
    if (proj > state_line_max_proj_.at(min_idx))
    {
      state_line_max_proj_.at(min_idx) = proj;
    }

  }

  geometry_msgs::PoseArray state_lines_array;
  state_lines_array.header.frame_id = "/odom";
  state_lines_array.header.stamp = latest_time_;

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "row_detection";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (int i = 0; i < state_lines.size(); ++i)
  {

    geometry_msgs::Point line_start, line_end;
    line_start.x = line_points.at(i)(0) + state_line_min_proj_.at(i)*line_directions.at(i)(0);
    line_start.y = line_points.at(i)(1) + state_line_min_proj_.at(i)*line_directions.at(i)(1);
    line_end.x   = line_points.at(i)(0) + state_line_max_proj_.at(i)*line_directions.at(i)(0);
    line_end.y   = line_points.at(i)(1) + state_line_max_proj_.at(i)*line_directions.at(i)(1);

    geometry_msgs::Pose line_start_pose, line_end_pose;
    line_start_pose.position.x = line_start.x;
    line_start_pose.position.y = line_start.y;
    line_end_pose.position.x = line_end.x;
    line_end_pose.position.y = line_end.y;

    state_lines_array.poses.push_back(line_start_pose);
    state_lines_array.poses.push_back(line_end_pose);

    marker.points.push_back(line_start);
    marker.points.push_back(line_end);

  }

  state_lines_pub_.publish(state_lines_array);

  markers_pub_.publish(marker);

}

} // namespace vineyard_localisation

