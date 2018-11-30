
#include <vineyard_localisation/ekf_lines.h>

namespace vineyard_localisation
{

EKFLines::EKFLines() : n_lines_(0)
{
  lines_sub_ = node_handle_.subscribe("/lines", 1, &EKFLines::linesCallback, this);
  line_markers_pub_ = node_handle_.advertise<visualization_msgs::Marker>("line_markers", 10);
  row_centre_pub_ = node_handle_.advertise<geometry_msgs::Point>("row_centre", 1);
}

void EKFLines::linesCallback(const geometry_msgs::PoseArray::ConstPtr & lines)
{

  // We didn't observe any lines.
  if (lines->poses.size() == 0)
  {
    return;
  }

  // Convert observed lines to matrix format.
  // XXX Convert naming to segments where appropriate.
  Eigen::MatrixXf observed_line_segments(6, lines->poses.size()/2);
  
  for (int i = 0; i < lines->poses.size(); i += 2)
  {

    // Start of the line segment.
    observed_line_segments(0,i/2) = lines->poses[i  ].position.x;
    observed_line_segments(1,i/2) = lines->poses[i  ].position.y;
    observed_line_segments(2,i/2) = lines->poses[i  ].position.z;

    // End of the line segment.
    observed_line_segments(3,i/2) = lines->poses[i+1].position.x;
    observed_line_segments(4,i/2) = lines->poses[i+1].position.y;
    observed_line_segments(5,i/2) = lines->poses[i+1].position.z;

  }

  // Convert line segments to polar coordinates.
  Eigen::MatrixXf observed_lines_polar(2, observed_line_segments.cols());

  for (int i = 0; i < observed_line_segments.cols(); ++i)
  {
    observed_lines_polar.col(i) = vineyard_row_detection::segmentToPolar(observed_line_segments.col(i));
  }

  // Get the robot pose, since data association is performed in the local coordinate frame.
  Eigen::Vector2f robot_pose(tx(), ty());

  std::vector<bool> is_valid_line = vineyard_row_detection::equispacedParallelLinesHough(observed_lines_polar, robot_pose, 10, 10);
  publishLineMarkers(observed_line_segments, is_valid_line);

  return;

  // Remove all of the invalid lines.
  Eigen::MatrixXf valid_observed_lines_polar;
  for (int i = 0; i < is_valid_line.size(); ++i)
  {
    if (is_valid_line[i])
    {
      valid_observed_lines_polar.conservativeResize(2, valid_observed_lines_polar.cols() + 1);
      valid_observed_lines_polar.col(valid_observed_lines_polar.cols() - 1) = observed_lines_polar.col(i);
    }
  }

  // Match to existing lines.
  int before = valid_observed_lines_polar.cols();
  valid_observed_lines_polar = existingLinesUpdate(valid_observed_lines_polar, robot_pose);

  removeLostLines();

//  ROS_INFO("%d %d", n_lines_, before - valid_observed_lines_polar.cols());

  // Match to potential lines.
  //ROS_INFO("IN");
  potentialLinesUpdate(valid_observed_lines_polar, robot_pose);
  //ROS_INFO("OUT");

  publishRowMarkers();

  publishRowCentre();

  rosInfoEigen(cov_.block<7,7>(0,0));

//  std::stringstream ss;
//  ss << std::endl << cov_ << std::endl;
//  ROS_INFO("%s", ss.str().c_str());

}

// Update based on observed lines that are matched to existing lines in the state.
// Return the lines that were not matched to those in the state.
Eigen::MatrixXf EKFLines::existingLinesUpdate(Eigen::MatrixXf & observed_lines_polar, const Eigen::Vector2f & robot_pose)
{

  // Don't do anything
  if (n_lines_ > 0)
  {

    // Convert lines in the state vector to matrix format.
    Eigen::MatrixXf existing_lines_polar(2, n_lines_);

    for (int i = 0; i < n_lines_; ++i)
    {
      existing_lines_polar(0,i) = state_(7 + 2*i    );
      existing_lines_polar(1,i) = state_(7 + 2*i + 1);
    }

    //ROS_INFO("START!");
    std::vector<int> matched_indices = vineyard_row_detection::associateLines(existing_lines_polar, observed_lines_polar, robot_pose);
    //ROS_INFO("END!");

//    std::stringstream ss;
//    ss << std::endl << existing_lines_polar << std::endl << observed_lines_polar << std::endl;
//
//    for (int i = 0; i < matched_indices.size(); ++i)
//    {
//      ss << matched_indices[i] << ' ';
//    }
//    ROS_INFO("%s", ss.str().c_str());

    for (int i = 0; i < n_iters_since_last_seen_.size(); ++i)
    {
      n_iters_since_last_seen_.at(i)++;
    }

    Eigen::MatrixXf unmatched_lines_polar;
    for (int i = 0; i < matched_indices.size(); ++i)
    {
      
      //ROS_INFO("i: %d, n_iters_size: %d", matched_indices[i], n_iters_since_last_seen_.size());
      //

      // If the line was not matched to any of the existing lines, add it to the lines to be return as unmatched.
      if (matched_indices[i] < 0)
      {
        unmatched_lines_polar.conservativeResize(2, unmatched_lines_polar.cols() + 1);
        unmatched_lines_polar.col(unmatched_lines_polar.cols() - 1) = observed_lines_polar.col(i);
        //ROS_INFO("BEFORE 1");
//        n_iters_since_last_seen_[matched_indices[i]]++;
        //ROS_INFO("AFTER 1");
      }
      // Line was matched to a line in the state vector, perform the EKF update.
      else
      {
        Eigen::Vector3f state_update = lineEKFUpdate(observed_lines_polar.col(i).cast<double>(), 7 + 2*matched_indices[i]);
        for (int j = i + 1; j < observed_lines_polar.cols(); ++j)
        {
//          observed_lines_polar(1,j) += state_update(2);
//          observed_lines_polar.col(j) = vineyard_row_detection::shiftPolarOrigin(observed_lines_polar.col(j), state_update.segment<2>(0));
        }
        //ROS_INFO("BEFORE 2");
        n_iters_since_last_seen_[matched_indices[i]] = 0;
        //ROS_INFO("AFTER 2");
      }

    }

    return unmatched_lines_polar;

  }
  else
  {
    return observed_lines_polar;
  }

}

Eigen::Vector3f EKFLines::lineEKFUpdate(const Eigen::Vector2d & line_meas, int state_idx)
{

  // XXX Measured radius and angle variables.

  Eigen::Vector2d line_exp = state_.segment<2>(state_idx);

  Eigen::Vector2d robot_pose = state_.segment<2>(0);
  Eigen::Vector2d line_meas_local = vineyard_row_detection::shiftPolarOrigin(line_meas.cast<float>(), robot_pose.cast<float>()).cast<double>();
  Eigen::Vector2d line_exp_local  = vineyard_row_detection::shiftPolarOrigin(line_exp.cast<float>(),  robot_pose.cast<float>()).cast<double>();

  line_meas_local = vineyard_row_detection::makeRadiusPositive(line_meas_local.cast<float>()).cast<double>();
  line_exp_local  = vineyard_row_detection::makeRadiusPositive(line_exp_local.cast<float>()).cast<double>();

  Eigen::Vector2d innovation = line_meas_local - line_exp_local;

//  std::stringstream ss;
//  ss << std::endl << line_meas_local << std::endl << line_exp_local << std::endl;
//  ROS_INFO("%s", ss.str().c_str());

  Eigen::MatrixXd meas_jacobian = Eigen::MatrixXd::Zero(2, size());
  meas_jacobian(0,0) = -cos(line_meas_local(1));
  meas_jacobian(0,1) = -sin(line_meas_local(1));
  meas_jacobian(0,state_idx) = 1.0;
  meas_jacobian(0,state_idx+1) = tx()*sin(line_meas_local(1)) - ty()*cos(line_meas_local(1));
  meas_jacobian(1,3) =  2.0*fabs(qw())/sqrt(qw()*qw() + qz()*qz());//-1.0/sqrt(1 - qw()*qw());
  meas_jacobian(1,6) = -2.0*fabs(qz())/sqrt(qw()*qw() + qz()*qz());
  meas_jacobian(1,state_idx+1) = 1.0;

  Eigen::Matrix2d meas_cov = Eigen::Matrix2d::Zero();
  meas_cov(0,0) = 0.1*0.1;
  meas_cov(1,1) = (10.0*M_PI/180.0)*(10.0*M_PI/180.0);

  Eigen::Matrix2d innovation_cov = meas_jacobian*cov_*meas_jacobian.transpose() + meas_cov;

  Eigen::MatrixXd kalman_gain = cov_*meas_jacobian.transpose()*innovation_cov.inverse();

  Eigen::VectorXd delta_state = kalman_gain*innovation;

  double prev_yaw = vineyard_row_detection::yawFromQuat(quat());

  state_ += delta_state; 

  normaliseQuat();

  double current_yaw = vineyard_row_detection::yawFromQuat(quat());

  //ROS_INFO("Here!");
  //rosInfoEigen(cov_.block<7,7>(0,0));
  //rosInfoEigen((Eigen::MatrixXd::Identity(size(), size()) - kalman_gain*meas_jacobian).block<7,7>(0,0));

  cov_ = (Eigen::MatrixXd::Identity(size(), size()) - kalman_gain*meas_jacobian)*cov_;
  //rosInfoEigen(cov_.block<7,7>(0,0));

  float delta_yaw = current_yaw - prev_yaw;

//  return delta_state.segment<2>(0).cast<float>();
  return Eigen::Vector3f(delta_state(0), delta_state(1), delta_yaw);

}

void EKFLines::removeLostLines()
{

  int n_iters_lost = 3;

  //ROS_INFO("1");
  //ROS_INFO("%d", n_iters_since_last_seen_.size());
  std::vector<bool> was_removed(n_iters_since_last_seen_.size());
  std::fill(was_removed.begin(), was_removed.end(), false);
  //ROS_INFO("2");

  // Removed elements from the state vector and covariance matrix if the corresponding line has not been seen for a certain amount of time.
  for (int i = n_iters_since_last_seen_.size() - 1; i >= 0; --i)
  {

    //ROS_INFO("LOST: %d", n_iters_since_last_seen_[i]);

    if (n_iters_since_last_seen_[i] >= n_iters_lost)
    {

      Eigen::VectorXd new_state(state_.rows() - 2);
      Eigen::MatrixXd new_cov(state_.rows() - 2, state_.rows() - 2);

      int n_before_end = state_.rows() - 7 - 2*i - 2;
      // The pose of the robot and the lines before this line.
      new_state.segment(0, 7 + 2*i) = state_.segment(0, 7 + 2*i);
      // The lines after this line.
      new_state.segment(7 + 2*i, n_before_end) = state_.segment(7 + 2*i + 2, n_before_end);

      // Upper left.
      new_cov.block(0, 0, 7 + 2*i, 7 + 2*i) = cov_.block(0, 0, 7 + 2*i, 7 + 2*i);
      // Lower left.
      new_cov.block(7 + 2*i, 0, n_before_end, 7 + 2*i) = cov_.block(7 + 2*i + 2, 0, n_before_end, 7 + 2*i);
      // Upper right.
      new_cov.block(0, 7 + 2*i, 7 + 2*i, n_before_end) = cov_.block(0, 7 + 2*i + 2, 7 + 2*i, n_before_end);
      // Lower left.
      new_cov.block(7 + 2*i, 7 + 2*i, n_before_end, n_before_end) = cov_.block(7 + 2*i + 2, 7 + 2*i + 2, n_before_end, n_before_end);

      state_ = new_state;
      cov_ = new_cov;

      n_lines_--;

      was_removed[n_iters_since_last_seen_.size() - i - 1] = true;

    }

  }

  //ROS_INFO("Remove: state_rows: %d, cov_rows: %d, cov_cols: %d, n_lines: %d", state_.rows(), cov_.rows(), cov_.cols(), n_lines_);

  // Copy the count of the iterations since the lines were last seen.
  std::vector<int> new_n_iters_since_last_seen;
  for (int i = 0; i < n_iters_since_last_seen_.size(); ++i)
  {
    if (!was_removed[i])
    {
      new_n_iters_since_last_seen.push_back(n_iters_since_last_seen_[i]);
    }
  }

  n_iters_since_last_seen_ = new_n_iters_since_last_seen;

}

void EKFLines::potentialLinesUpdate(const Eigen::MatrixXf & observed_lines_polar, const Eigen::Vector2f & robot_pose)
{

  //ROS_INFO("Potentual Lines Update");

  int score_init = 2;

  // Check we have any potential lines currently.
  if (potential_lines_.cols() > 0)
  {

    // Associate observed line to potential lines.
    std::vector<int> matched_indices = vineyard_row_detection::associateLines(potential_lines_, observed_lines_polar, robot_pose);

    std::vector<int> matched_indices_in_observed(potential_lines_.cols());
    std::fill(matched_indices_in_observed.begin(), matched_indices_in_observed.end(), -1);

    // Increment the scores of all of the observed potential lines.
    for (int i = 0; i < matched_indices.size(); ++i)
    {

      // This line was not matched.
      if (matched_indices[i] < 0) { continue; }

      potential_lines_scores_[matched_indices[i]]++;
      matched_indices_in_observed[matched_indices[i]] = i;
    }

    // Decrement the scores of all of the unobserved potential lines.
    for (int i = 0; i < matched_indices_in_observed.size(); ++i)
    {
      if (matched_indices_in_observed[i] < 0)
      {
        potential_lines_scores_[i]--;
      }
    }

    // Add potential lines to the state if they are above the add threshold, remove lines from the state if they are below the remove threshold, otherwise do nothing.
    int score_thresh_add = 4, score_thresh_remove = 0;
    Eigen::MatrixXf confirmed_lines, potential_lines;
    std::vector<int> potential_lines_scores;

    for (int i = 0; i < potential_lines_scores_.size(); ++i)
    {

      // This line has been confirmed as existing, we want to add it to the state vector.
      if (potential_lines_scores_[i] >= score_thresh_add)
      {
        confirmed_lines.conservativeResize(2, confirmed_lines.cols() + 1);
        confirmed_lines(0, confirmed_lines.cols() - 1) = potential_lines_(0,i); 
        confirmed_lines(1, confirmed_lines.cols() - 1) = potential_lines_(1,i); 
      }
      // This line has neither been confirmed or denied, keep it as a potential line.
      else if (potential_lines_scores_[i] > score_thresh_remove)
      {
        potential_lines.conservativeResize(2, potential_lines.cols() + 1);
        // Copy the matched line if it was matched.
        if (matched_indices_in_observed[i] >= 0)
        {
          potential_lines(0, potential_lines.cols() - 1) = observed_lines_polar(0, matched_indices_in_observed[i]);//potential_lines_(0,i); 
          potential_lines(1, potential_lines.cols() - 1) = observed_lines_polar(1, matched_indices_in_observed[i]);//potential_lines_(1,i); 
        }
        else
        {
          potential_lines(0, potential_lines.cols() - 1) = potential_lines_(0,i); 
          potential_lines(1, potential_lines.cols() - 1) = potential_lines_(1,i); 
        }
        potential_lines_scores.push_back(potential_lines_scores_[i]);
      }

    }

    // Add the confirmed lines to the state vector.    
    addLinesToState(confirmed_lines);

    // Add observed that were not matched to potential lines to the potential lines.
    for (int i = 0; i < matched_indices.size(); ++i)
    {

      if (matched_indices[i] < 0)
      {
        potential_lines.conservativeResize(2, potential_lines.cols() + 1);
        potential_lines.col(potential_lines.cols() - 1) = observed_lines_polar.col(i);
        potential_lines_scores.push_back(score_init);
      }

    }

    potential_lines_ = potential_lines;
    potential_lines_scores_ = potential_lines_scores;

    //ROS_INFO("plines: %d, plinesscores: %d", potential_lines_.cols(), potential_lines_scores_.size());

  }
  // If don't have any potential lines, all of our observed lines become potential lines.
  else
  {
    potential_lines_ = observed_lines_polar;
    potential_lines_scores_.resize(potential_lines_.cols());
    std::fill(potential_lines_scores_.begin(), potential_lines_scores_.end(), score_init);
  }

}

void EKFLines::addLinesToState(const Eigen::MatrixXf & new_lines_polar)
{

  // XXX Make functions for resizing matrices.
  // Add the lines to the state vector.
  int old_state_length = state_.rows();
  state_.conservativeResize(state_.rows() + 2*new_lines_polar.cols());
  Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(state_.rows(), state_.rows());
  new_cov.block(0, 0, old_state_length, old_state_length) = cov_;
  cov_ = new_cov;

  int j = 0;
  for (int i = old_state_length; i < state_.rows(); i += 2, ++j)
  {
//    ROS_INFO("New line added to state!");
    state_(i  ) = new_lines_polar(0,j);
    state_(i+1) = new_lines_polar(1,j);
    //cov_(i  ,i  ) = 0.5*0.5;
    //cov_(i+1,i+1) = (30.0*M_PI/180.0)*(30.0*M_PI/180.0);
    cov_.block(i, i, 2, 2) = initLineCovariance(new_lines_polar.col(j).cast<double>());
    n_iters_since_last_seen_.push_back(0);
  }

  n_lines_ += new_lines_polar.cols();

  //ROS_INFO("n_lines: %d, state_rows: %d", n_lines_, state_.rows());

}

Eigen::Matrix2d EKFLines::initLineCovariance(const Eigen::Vector2d & new_line_polar)
{

  Eigen::MatrixXd meas_jacobian = Eigen::MatrixXd::Zero(2, 9);
  meas_jacobian(0,0) = -cos(new_line_polar(1));
  meas_jacobian(0,1) = -sin(new_line_polar(1));
  meas_jacobian(0,7) = 1.0;
  meas_jacobian(0,8) = tx()*sin(new_line_polar(1)) - ty()*cos(new_line_polar(1));
  meas_jacobian(1,3) =  2.0*fabs(qw())/sqrt(qw()*qw() + qz()*qz());//-1.0/sqrt(1 - qw()*qw());
  meas_jacobian(1,6) = -2.0*fabs(qz())/sqrt(qw()*qw() + qz()*qz());
  meas_jacobian(1,8) = 1.0;

  Eigen::Matrix2d meas_cov;
  meas_cov << 0.1*0.1,                                 0.0,
                  0.0, (10.0*M_PI/180.0)*(10.0*M_PI/180.0);

  Eigen::MatrixXd cov_with_line = Eigen::MatrixXd::Zero(9, 9);
  cov_with_line.block<7,7>(0,0) = cov_.block<7,7>(0,0);
  cov_with_line.block<2,2>(7,7) = meas_cov;

//  std::stringstream ss;
//  ss << std::endl << meas_jacobian*cov_with_line*meas_jacobian.transpose() << std::endl;
//  ROS_INFO("%s", ss.str().c_str());

  return meas_cov;//meas_jacobian*cov_with_line*meas_jacobian.transpose();

}

void EKFLines::publishLineMarkers(const Eigen::MatrixXf & lines_coefficients, const std::vector<bool> & is_valid_line)
{

  int count = 0;

  for (int i = 0; i < lines_coefficients.cols(); ++i)
  {

    visualization_msgs::Marker line_markers;

    line_markers.ns = "vineyard_lines";
    line_markers.id = count++;
    line_markers.type = visualization_msgs::Marker::LINE_LIST;
    line_markers.action= visualization_msgs::Marker::ADD;
    line_markers.header.frame_id = "/odom";
    line_markers.scale.x = 0.05;
    line_markers.color.a = 1.0;
    line_markers.lifetime = ros::Duration();

    geometry_msgs::Point line_start, line_end;

    if (is_valid_line[i])
    {
      line_markers.color.r = 1.0;
      line_markers.color.g = 1.0;
      line_markers.color.b = 0.0;
    }
    else
    {
      line_markers.color.r = 1.0;
      line_markers.color.g = 0.0;
      line_markers.color.b = 0.0;
    }

    line_start.x = lines_coefficients(0,i);
    line_start.y = lines_coefficients(1,i);
    line_start.z = lines_coefficients(2,i);

    line_end.x = lines_coefficients(3,i);
    line_end.y = lines_coefficients(4,i);
    line_end.z = lines_coefficients(5,i);

    line_markers.points.push_back(line_start);
    line_markers.points.push_back(line_end);

    line_markers_pub_.publish(line_markers);

  }

}

void EKFLines::publishRowMarkers()
{

  visualization_msgs::Marker row_markers;
  row_markers.ns = "vineyard_rows";
  row_markers.id = 0;
  row_markers.type = visualization_msgs::Marker::LINE_LIST;
  row_markers.action= visualization_msgs::Marker::ADD;
  row_markers.header.frame_id = "/odom";
  row_markers.scale.x = 0.25;
  row_markers.color.a = 1.0;
  row_markers.lifetime = ros::Duration();
  row_markers.color.r = 0.0;
  row_markers.color.g = 0.0;
  row_markers.color.b = 1.0;


  // Visualise the rows by projecting the origin and the robot pose onto the line.
  for (int i = 0; i < n_lines_; ++i)
  {

    geometry_msgs::Point row_start, row_end;

    Eigen::Vector2f row_start_eigen = vineyard_row_detection::projectPointOntoPolarLine(state_.segment<2>(7 + 2*i).cast<float>(), Eigen::Vector2f(0.0, 0.0));
    Eigen::Vector2f row_end_eigen   = vineyard_row_detection::projectPointOntoPolarLine(state_.segment<2>(7 + 2*i).cast<float>(), Eigen::Vector2f((float)tx(), (float)ty()));

    row_start.x = row_start_eigen(0);
    row_start.y = row_start_eigen(1);
    row_start.z = 0.0;

    row_end.x = row_end_eigen(0);
    row_end.y = row_end_eigen(1);
    row_end.z = tz();

    row_markers.points.push_back(row_start);
    row_markers.points.push_back(row_end);

  }

  // XXX Rename publisher.
  // XXX Make all vector accessess .at()
  line_markers_pub_.publish(row_markers);

}

void EKFLines::publishRowCentre()
{

  // Find the centre of the row that we are currently in.
  // XXX Need be management of and access to lines in the state vector.
  Eigen::Vector2f first_row_side, second_row_side;

  // Check if there is a candidate in the state vector.
  int first_idx = -1, second_idx = -1;
  for (int i = 0; i < n_lines_; ++i)
  {
    
    Eigen::Vector2f line_polar = state_.segment<2>(7 + 2*i).cast<float>();

    Eigen::Vector2f line_polar_local = vineyard_row_detection::shiftPolarOrigin(line_polar, Eigen::Vector2f(tx(), ty()));
    line_polar_local = vineyard_row_detection::makeRadiusPositive(line_polar_local);

    // Should have radius less than 2m.
    if (line_polar_local(0) < 2.0)
    {
      if (first_idx < 0) { first_idx  = i; }
      else               { second_idx = i; }
    }

  }

  // XXX Move up, don't need -1.
  bool first_idx_done = false, second_idx_done = false;

  if (first_idx >= 0)
  {
    first_row_side = state_.segment<2>(7 + 2*first_idx).cast<float>();
    first_idx_done = true;
  }
  if (second_idx >= 0)
  {
    second_row_side = state_.segment<2>(7 + 2*second_idx).cast<float>();
    second_idx_done = true; 
  }

  // Check in the potential lines if we haven't found both sides of the row.
  // XXX Proper matrix sizes - not Xf.
  // XXX Don't store lines in matrices, make class.
  if (!(first_idx > 0 && second_idx > 0))
  {
    for (int i = 0; i < potential_lines_.cols(); ++i)
    {

      Eigen::Vector2f line_polar_local = vineyard_row_detection::shiftPolarOrigin(potential_lines_.col(i), Eigen::Vector2f(tx(), ty()));
      line_polar_local = vineyard_row_detection::makeRadiusPositive(line_polar_local);

      // Should have radius less than 2m.
      if (line_polar_local(0) < 2.0)
      {
        if (first_idx < 0) { first_idx  = i; }
        else               { second_idx = i; }
      }

    }
  }

  if (!first_idx_done && first_idx >= 0)
  {
    first_row_side = potential_lines_.col(first_idx);
    first_idx_done = true;
  }
  if (!second_idx_done && second_idx >= 0)
  {
    second_row_side = potential_lines_.col(second_idx);
    second_idx_done = true;
  }

  // Don't do anything if we haven't found both sides of the row that we're in.
  if (!(first_idx_done && second_idx_done))
  {
    return;
  }

  // The rows will be close to parallel, modify the second if their angles are too far apart.
  else
  {
    float delta_angle = first_row_side(1) - second_row_side(1);
    if (delta_angle >  M_PI/4.0) { second_row_side(1) += 2.0*M_PI; }
    if (delta_angle < -M_PI/4.0) { second_row_side(1) -= 2.0*M_PI; }
  }

  // Take the average of the 2 sides.
  float row_centre_radius = (first_row_side(0) + second_row_side(0))/2.0;
  float row_centre_angle  = (first_row_side(1) + second_row_side(1))/2.0;

  Eigen::Vector2f row_centre(row_centre_radius, row_centre_angle);

  visualization_msgs::Marker row_centre_marker;
  row_centre_marker.ns = "vineyard_row_centre";
  row_centre_marker.id = 0;
  row_centre_marker.type = visualization_msgs::Marker::LINE_LIST;
  row_centre_marker.action= visualization_msgs::Marker::ADD;
  row_centre_marker.header.frame_id = "/odom";
  row_centre_marker.scale.x = 0.25;
  row_centre_marker.color.a = 1.0;
  row_centre_marker.lifetime = ros::Duration();
  row_centre_marker.color.r = 1.0;
  row_centre_marker.color.g = 1.0;
  row_centre_marker.color.b = 1.0;


  // Visualise the rows by projecting the origin and the robot pose onto the line.
  geometry_msgs::Point row_start, row_end;

  Eigen::Vector2f row_start_eigen = vineyard_row_detection::projectPointOntoPolarLine(row_centre, Eigen::Vector2f(0.0, 0.0));
  Eigen::Vector2f row_end_eigen   = vineyard_row_detection::projectPointOntoPolarLine(row_centre, Eigen::Vector2f((float)tx(), (float)ty()));

  row_start.x = row_start_eigen(0);
  row_start.y = row_start_eigen(1);
  row_start.z = 0.0;

  row_end.x = row_end_eigen(0);
  row_end.y = row_end_eigen(1);
  row_end.z = tz();

  row_centre_marker.points.push_back(row_start);
  row_centre_marker.points.push_back(row_end);

  line_markers_pub_.publish(row_centre_marker);

  geometry_msgs::Point row_centre_polar;
  row_centre_polar.x = row_centre_radius;
  row_centre_polar.z = row_centre_angle;

  row_centre_pub_.publish(row_centre_polar);

}

void EKFLines::rosInfoEigen(const Eigen::MatrixXd & mat)
{

  std::stringstream ss;
  ss << std::endl << mat << std::endl;
  ROS_INFO("%s", ss.str().c_str());

}

} // namespace vineyard_localisation

