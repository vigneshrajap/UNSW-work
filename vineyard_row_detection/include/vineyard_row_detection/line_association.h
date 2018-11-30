
#ifndef LINE_ASSOCIATION_H_
#define LINE_ASSOCIATION_H_

#include <vineyard_row_detection/line_utils.h>

// XXX CONVERT ALL LINE CODE TO DOUBLE.
// XXX Create .cpp files.

namespace vineyard_row_detection
{

// Return a vector of the indices in the original lines of each matched new line.
// Each column of the matrices represents a line in global polar coordinates.
// A negative index indicates that the lines was not matched. 
inline std::vector<int> associateLines(const Eigen::MatrixXf & existing_lines, const Eigen::MatrixXf & new_lines, const Eigen::Vector2f & pose)
{

  // Don't do anything if there are no new lines.
  if (new_lines.cols() == 0) { return std::vector<int>(0); }

  // Convert both the new and existing lines to local pose-relative coordinates.
  Eigen::MatrixXf existing_lines_local(existing_lines.rows(), existing_lines.cols()), new_lines_local(new_lines.rows(), new_lines.cols());

  for (int i = 0; i < existing_lines.cols(); ++i)
  {
    existing_lines_local.col(i) = shiftPolarOrigin(existing_lines.col(i), pose);
    existing_lines_local.col(i) = makeRadiusPositive(existing_lines_local.col(i));
  }

  for (int i = 0; i < new_lines.cols(); ++i)
  {
    new_lines_local.col(i) = shiftPolarOrigin(new_lines.col(i), pose);
    new_lines_local.col(i) = makeRadiusPositive(new_lines_local.col(i));
  }

  // For each existing line, find the closest new line based on a weighted Euclidean distance of the local polar coordinates of the lines.
  float weight = 0.05;
  std::vector<int> matched_indices(new_lines.cols());
  std::fill(matched_indices.begin(), matched_indices.end(), -1);
  int matched_idx;

//  std::stringstream ss;
//  ss << std::endl << existing_lines_local << std::endl << new_lines_local;
//  ROS_INFO("%s", ss.str().c_str());

  for (int i = 0; i < existing_lines_local.cols(); ++i)
  {

    float existing_radius = existing_lines_local(0,i);
    float existing_angle  = existing_lines_local(1,i);

    float min_dist = std::numeric_limits<float>::max();
    float min_dist_thresh = 0.5;

    for (int j = 0; j < new_lines_local.cols(); ++j)
    {

      float new_radius = new_lines_local(0,j);
      float new_angle  = new_lines_local(1,j);

      float delta_radius = existing_radius - new_radius;
      float delta_angle = normaliseAngle(existing_angle - new_angle);

      float dist = sqrt(delta_radius*delta_radius + weight*delta_angle*delta_angle);

      if (dist < min_dist)
      {
        matched_idx = j;
        min_dist = dist;
      }

    }

//    ROS_INFO("%f", min_dist);

    // XXX handle case where multiple existing lines are associated to the same new line.
    // Add the match if it is below the theshold.
    if (min_dist < min_dist_thresh)
    {
      matched_indices[matched_idx] = i;
    }

  }

  return matched_indices;

}

// XXX Move to own file?
inline std::vector<bool> equispacedParallelLinesHough(const Eigen::MatrixXf & lines, const Eigen::Vector2f & pose, int radius_bins, int angle_bins)
{

  // Convert the lines to local coordinates.
  Eigen::MatrixXf lines_local(lines.rows(), lines.cols());

  for (int i = 0; i < lines.cols(); ++i)
  {
    lines_local.col(i) = shiftPolarOrigin(lines.col(i), pose);
  }

  // Optimum spacing.
  float max_delta_radius = 5.0;
  float radius_spacing = max_delta_radius/(float)radius_bins;
  float angle_spacing = M_PI/(float)angle_bins;
  // XXX
  //Eigen::MatrixXi hough_acc = Eigen::MatrixXi::Zero(radius_bins, angle_bins);
  //std::vector<Eigen::MatrixXi> was_hough_acced(lines.cols());
  //std::fill(was_hough_acced.begin(), was_hough_acced.end(), Eigen::MatrixXi::Zero(radius_bins, angle_bins));
  Eigen::VectorXi angle_acc = Eigen::VectorXi::Zero(angle_bins);
  std::vector<Eigen::VectorXi> was_angle_acced(lines.cols());
  std::fill(was_angle_acced.begin(), was_angle_acced.end(), Eigen::VectorXi::Zero(angle_bins));

  for (int i = 0; i < lines_local.cols(); ++i)
  {
    for (int j = 0; j < lines_local.cols(); ++j)
    {

      // Don't compare a line against itself.
      if (i == j) { continue; }

      float radius_1 = lines_local(0,i);
      float angle_1  = lines_local(1,i);
      float radius_2 = lines_local(0,j);
      float angle_2  = lines_local(1,j);

      float delta_angle = normaliseAngle(angle_1 - angle_2);

      // Don't include lines if they are more than 20 degrees apart.
      if (fabs(delta_angle) > M_PI*20.0/180.0) { continue; }

      // Transform radius_2 and angle_2 so that the cosine of angle_2 relative to angle_1 is positive (delta_angle will be in quadrants 1 or 4).
      if (cos(delta_angle) < 0.0)
      {

        if (angle_2 > 0.0) { angle_2 -= M_PI; }
        else               { angle_2 += M_PI; }

        radius_2 *= -1.0;

      }

      float delta_radius = fabs(radius_1 - radius_2);

      // Don't include lines if they are more than a certain distance apart.
      if (delta_radius >= max_delta_radius) { continue; }

      // XXX Hough
      // int radius_bin = (int)floor(delta_radius/radius_spacing);

      // Accumulate angle in [0, pi].
      // XXX are the angles normalised before entering this function?
      if (angle_1 < 0.0) { angle_1 += M_PI; }
      if (angle_2 < 0.0) { angle_2 += M_PI; }

      int angle_bin_1 = (int)floor(angle_1/angle_spacing);
      int angle_bin_2 = (int)floor(angle_2/angle_spacing);

      // XXX Hough
      // hough_acc(radius_bin, angle_bin_1)++;
      // hough_acc(radius_bin, angle_bin_2)++;

      // XXX Need angle_bin_2 as well?
      // XXX Hough
      // was_hough_acced[i](radius_bin, angle_bin_1) = 1;

      // XXX Angle only.
      // We know the lines should be 3m apart.
      if (delta_radius > 3.25 || delta_radius < 2.75) { continue; }

      angle_acc(angle_bin_1)++;
      was_angle_acced[i](angle_bin_1) = 1;

    }
  }

  // Find the peak in the Hough space.
  int max_r, max_c, max_hough = 0;
  for (int r = 0; r < angle_acc.rows(); ++r)
  {
    //for (int c = 0; c < angle_acc.cols(); ++c)
    //{
      if (angle_acc(r)/* XXX hough_acc(r,c)*/ > max_hough)
      {
        max_r = r;
    //    max_c = c;
        max_hough = angle_acc(r)/* XXX hough_acc(r,c)*/;
      }
    //}
  }

  std::vector<bool> is_valid_line(lines.cols());
  std::fill(is_valid_line.begin(), is_valid_line.end(), false);

  // No valid lines.
  if (max_hough == 0) { return is_valid_line; }

  for (int i = 0; i < /* XXX was_hough_acced*/was_angle_acced.size(); ++i)
  {
    if (/*was_hough_acced*/was_angle_acced[i](max_r/*,max_c*/) > 0)
    {
      is_valid_line[i] = true;
    }
  }

  return is_valid_line;

}

} // namespace vineyard_row_detection

#endif // #ifndef LINE_ASSOCIATION_H_

