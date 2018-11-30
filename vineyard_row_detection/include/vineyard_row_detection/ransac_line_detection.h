
#ifndef VITI_RANSAC_LINE_DETECTION_H_
#define VITI_RANSAC_LINE_DETECTION_H_

#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <vineyard_row_detection/line.h>

namespace vineyard_row_detection
{

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<point_t> point_cloud_t;
typedef point_cloud_t::Ptr point_cloud_ptr_t;
typedef pcl::SampleConsensusModelLine<point_t> line_model_t;
typedef line_model_t::Ptr line_model_ptr_t;
typedef pcl::RandomSampleConsensus<point_t> ransac_t;
typedef ransac_t::Ptr ransac_ptr_t;

std::vector<Line> RANSACDetectLines(point_cloud_ptr_t & cloud, int n_lines);

void removePointsFromCloud(point_cloud_t & cloud, const std::vector<int> & inliers);

} // namespace vineyard_row_detection

#endif // #ifndef VITI_RANSAC_LINE_DETECTION_H_

