
#ifndef VITI_EKF_LINES_H_
#define VITI_EKF_LINES_H_
// XXX Check all header guards.

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <vineyard_localisation/ekf_gps.h>

#include <vineyard_row_detection/line_association.h>

namespace vineyard_localisation
{

class EKFLines : public EKFGPS
{

public:

  EKFLines();

protected:

  // XXX Move size_ to base class.
  // XXX Get size from length of state_?
  // XXX inline all?
  int size() { return 7 + 2*n_lines_; }

private:

  void linesCallback(const geometry_msgs::PoseArray::ConstPtr & lines);

  Eigen::MatrixXf existingLinesUpdate(Eigen::MatrixXf & observed_lines_polar, const Eigen::Vector2f & robot_pose);

  Eigen::Vector3f lineEKFUpdate(const Eigen::Vector2d & line_meas, int state_idx);

  void removeLostLines();

  void potentialLinesUpdate(const Eigen::MatrixXf & observed_lines_polar, const Eigen::Vector2f & robot_pose);

  void addLinesToState(const Eigen::MatrixXf & new_lines_polar);

  Eigen::Matrix2d initLineCovariance(const Eigen::Vector2d & new_line_polar);

  void publishLineMarkers(const Eigen::MatrixXf & line_segments, const std::vector<bool> & is_valid_line);

  void publishRowMarkers();

  void publishRowCentre();

  void rosInfoEigen(const Eigen::MatrixXd & mat);

  ros::Subscriber lines_sub_;

  ros::Publisher line_markers_pub_;

  ros::Publisher row_centre_pub_;

  Eigen::MatrixXf potential_lines_;

  std::vector<int> potential_lines_scores_;

  std::vector<int> n_iters_since_last_seen_;

  int n_lines_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_LINES_H_

