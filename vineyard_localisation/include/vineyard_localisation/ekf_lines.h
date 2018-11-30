
#ifndef VITI_EKF_LINES_H_
#define VITI_EKF_LINES_H_

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <vineyard_localisation/ekf_gps.h>

#include <row_detection/line.h>

#include <point_cloud_assembler/point_cloud_assembler.h>

namespace vineyard_localisation
{

typedef row_detection::Line line_t;
typedef std::vector<line_t> line_vec_t;
typedef line_vec_t::iterator line_vec_it_t;
typedef line_vec_t::const_iterator line_vec_const_it_t;

class EKFLines : public EKFGPS
{

public:

  EKFLines();

  void init();

private:

  void linesCallback(const geometry_msgs::PoseArray::ConstPtr & lines);

  void matchLines(const line_vec_t & lines, std::vector<std::pair<int, int> > & state_indices, std::vector<std::pair<int, int> > & candidate_indices, std::vector<int> & unmatched_indices);

  void handleStateLines(line_vec_t & lines, const std::vector<std::pair<int, int> > & indices);

  void ekfUpdate(line_t & observed_line, line_t & expected_line, int state_line_idx, line_vec_t & lines);

  Eigen::MatrixXd lineCovariance(const line_t & line, int line_idx);

  void handleCandidateLines(const line_vec_t & lines, const std::vector<std::pair<int, int> > & indices);

  void addLineToState(line_t & new_state_line);

  Eigen::MatrixXd newLineCovariance();

  void handleUnmatchedLines(const line_vec_t & lines, const std::vector<int> & indices);

//  void publishStateLines(const geometry_msgs::PoseArray & lines, const std::vector<std::pair<int, int> > & state_indices);

  double lineToLineDistance(const line_t & line_1, const line_t & line_2);

  line_vec_t linesFromState();

  void assemblePointCloud(const ros::TimerEvent &);

  void findRowEndpoints(const pcl::PointCloud<pcl::PointXYZ> & cloud);

  callback_sequencer::CallbackSequencer<nav_msgs::Odometry, sensor_msgs::Imu, nav_msgs::Odometry, geometry_msgs::PoseArray> line_sequencer_;

  message_filters::Subscriber<geometry_msgs::PoseArray> lines_sub_;

  point_cloud_assembler::PointCloudAssembler point_cloud_assembler_;

  ros::Timer point_cloud_assemble_timer_;

  ros::Publisher state_lines_pub_;

  ros::Publisher markers_pub_;

  line_vec_t candidate_lines_;

  std::vector<int> candidate_line_counts_;

  std::vector<Eigen::Vector2d> line_endpoints_;

  std::vector<double> state_line_min_proj_;

  std::vector<double> state_line_max_proj_;

  double radius_weight_;

  double angle_weight_;

  double line_match_threshold_;

  int candidate_start_value_;

  int candidate_remove_value_;

  int candidate_add_value_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_EKF_LINES_H_
 
