
#include <fastslam/odometry_with_lines.h>

namespace fastslam
{

OdometryWithLines::OdometryWithLines()
{

  lines_sub_ = node_handle_.subscribe("lines", 1, &OdometryWithLines::linesCallback, this);

  state_with_lines_ = Eigen::VectorXf::Zero(6);
  cov_with_lines_   = Eigen::MatrixXf::Zero(6,6);

}

void OdometryWithLines::linesCallback(const geometry_msgs::PoseArray::ConstPtr & lines)
{


}

} // namespace fastslam

