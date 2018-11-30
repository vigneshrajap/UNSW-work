
#ifndef LINE_DETECTION_ITERATIVE_H_
#define LINE_DETECTION_ITERATIVE_H_

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <row_detection/line_detection_base.h>

namespace row_detection
{

typedef pcl::SampleConsensusModelLine<point_t> line_model_t;
typedef line_model_t::Ptr line_model_ptr_t;

class LineDetectionIterative : public LineDetectionBase
{

public:

  void setRadiusMin(double radius_min)
  {
    radius_min_ = radius_min;
  }

  void setRadiusMax(double radius_max)
  {
    radius_max_ = radius_max;
  }

  void setRadiusSamples(int radius_samples)
  {
    radius_samples_ = radius_samples;
  }

  void setAngleMin(double angle_min)
  {
    angle_min_ = angle_min;
  }

  void setAngleMax(double angle_max)
  {
    angle_max_ = angle_max;
  }

  void setAngleSamples(int angle_samples)
  {
    angle_samples_ = angle_samples;
  }

  void setNumberOfIterations(int n_iters)
  {
    n_iters_ = n_iters;
  }

  void setDistanceThreshold(double dist_thresh)
  {
    dist_thresh_ = dist_thresh;
  }

  bool compute();

  void convertPointCloudToCentroidCoordinates();

  void generateLineSamples();

  bool fitLine(Line & line);

  void findLineSegments();

  line_segment_t findLineSegment(const Line & line, const cloud_t & cloud);

  void saveLineSegments(const std::string & filename);

private:

  line_model_ptr_t line_model_;

  double radius_min_;

  double radius_max_;

  int radius_samples_;

  double angle_min_;

  double angle_max_;

  int angle_samples_;

  int n_iters_;

  double dist_thresh_;

  double centroid_x_;

  double centroid_y_;

  double centroid_z_;

};

} // namespace row_detection

#endif // #ifndef LINE_DETECTION_ITERATIVE_H_

