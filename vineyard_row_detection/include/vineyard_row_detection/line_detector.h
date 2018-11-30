
#ifndef LINE_DETECTOR_H_
#define LINE_DETECTOR_H_

#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

namespace vineyard_row_detection
{

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<point_t> point_cloud_t;
typedef point_cloud_t::Ptr point_cloud_ptr_t;
typedef pcl::SampleConsensusModelLine<point_t> line_model_t;
typedef line_model_t::Ptr line_model_ptr_t;
typedef pcl::RandomSampleConsensus<point_t> ransac_t;
typedef ransac_t::Ptr ransac_ptr_t;

class LineDetector
{

public:

  LineDetector();

  void setInputCloud(point_cloud_ptr_t & cloud)
  {
    cloud_ = cloud;
    line_model_->setInputCloud(cloud_);
  }

  void detectLines(Eigen::Matrix<float, 6, 10> & lines_coefficients);

private:

  void findStartAndEndPoints(const point_cloud_t & line_cloud, Eigen::VectorXf & line_coefficients);

  ransac_ptr_t ransac_;

  line_model_ptr_t line_model_;

  point_cloud_ptr_t cloud_;

};

} // namespace vineyard_row_detection

#endif // #ifndef LINE_DETECTOR_H_

