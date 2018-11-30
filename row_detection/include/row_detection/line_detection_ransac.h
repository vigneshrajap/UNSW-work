
#ifndef LINE_DETECTION_RANSAC_H_
#define LINE_DETECTION_RANSAC_H_

#include <pcl/common/io.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <row_detection/line_detection_base.h>

namespace row_detection
{

typedef pcl::SampleConsensusModelLine<point_t> line_model_t;
typedef line_model_t::Ptr line_model_ptr_t;
typedef pcl::RandomSampleConsensus<point_t> ransac_t;

class LineDetectionRANSAC : public LineDetectionBase
{

public:

  void setNumberOfLines(int n_lines)
  {
    n_lines_ = n_lines;
  }

  void setDistanceThreshold(double dist_thresh)
  {
    dist_thresh_ = dist_thresh;
  }

  bool compute();

  void removePointsFromCloud(const std::vector<int> & inliers);

private:

  line_segment_t findLineSegment(const Line & line, const cloud_t & cloud);

  int n_lines_;

  double dist_thresh_;

};

} // namespace row_detection

#endif // #ifndef LINE_DETECTION_RANSAC_H_

