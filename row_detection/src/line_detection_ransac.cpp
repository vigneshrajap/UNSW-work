
#include <row_detection/line_detection_ransac.h>

namespace row_detection
{

bool LineDetectionRANSAC::compute()
{

  /*
  cloud_t downsampled_cloud;
  pcl::VoxelGrid<point_t> voxel_grid;
  voxel_grid.setInputCloud(cloud_);
  voxel_grid.setLeafSize(0.5f, 0.5f, 100.0f);
  voxel_grid.filter(downsampled_cloud);
//  pcl::copyPointCloud(downsampled_cloud, *cloud_);
  */

  // Shift point cloud to local coordinates.
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (cloud_it_t point = cloud_->points.begin(); point != cloud_->points.end(); ++point)
  {
    sum_x += point->x;
    sum_y += point->y;
    sum_z += point->z;
  }

  centroid_x_ = sum_x/(double)cloud_->points.size();
  centroid_y_ = sum_y/(double)cloud_->points.size();
  centroid_z_ = sum_z/(double)cloud_->points.size();

  for (cloud_it_t point = cloud_->points.begin(); point != cloud_->points.end(); ++point)
  {
    point->x -= centroid_x_;
    point->y -= centroid_y_;
    point->z -= centroid_z_;
  }

  // Line model and RANSAC.
  line_model_ptr_t line_model = line_model_ptr_t(new line_model_t(cloud_));
  ransac_t line_ransac(line_model);
  line_model->setInputCloud(cloud_);
  line_ransac.setDistanceThreshold(dist_thresh_);
  
  // Clear the existing line segments.
  line_segments_.clear();

  for (int i = 0; i < n_lines_; ++i)
  {

    if (cloud_->points.size() < 2)
    {
      return false;
    }

    // Perform RANSAC.
    line_ransac.computeModel();
    line_ransac.refineModel();

    // Get the model coefficients.
    Eigen::VectorXf line_coefficients;
    line_ransac.getModelCoefficients(line_coefficients);

    // Get the inliers to the refined model.
    std::vector<int> inliers;
    line_model->selectWithinDistance(line_coefficients, dist_thresh_, inliers);

    // Add the line.
    lines_.push_back((Eigen::Matrix<double,6,1>)line_coefficients.cast<double>());

    // Get the inliers to the point cloud and extract the line segment.
    cloud_t inlier_cloud;
    pcl::copyPointCloud(*cloud_, inliers, inlier_cloud);
    line_segments_.push_back(findLineSegment(lines_.back(), inlier_cloud));

    // Remove the detected points from the cloud.
    removePointsFromCloud(inliers);

    // XXX Hack to get the indices to clear properly.
    line_model->getIndices()->clear();
    line_model->setInputCloud(cloud_);

  }

  return true;

}

void LineDetectionRANSAC::removePointsFromCloud(const std::vector<int> & inliers)
{

  // Find outliers.
  std::vector<int> outliers(cloud_->points.size() - inliers.size());

  for (size_t cloud_idx = 0, inlier_idx = 0, outlier_idx = 0; cloud_idx < cloud_->points.size(); ++cloud_idx)
  {
    if (inlier_idx >= inliers.size() || inliers.at(inlier_idx) != cloud_idx)
    {
      outliers.at(outlier_idx++) = cloud_idx;
    }
    else
    {
      ++inlier_idx;
    }
  }

  // Remove the points.
  cloud_ptr_t outlier_cloud(new cloud_t());
  pcl::copyPointCloud(*cloud_, outliers, *outlier_cloud);
  cloud_ = outlier_cloud;

}

Eigen::Matrix<double,6,1> LineDetectionRANSAC::findLineSegment(const Line & line, const cloud_t & cloud)
{

  double min_dist = std::numeric_limits<double>::max();
  double max_dist = -std::numeric_limits<double>::max();

  Eigen::Vector3d min_proj, max_proj;

  for (cloud_const_it_t point = cloud.points.begin(); point != cloud.points.end(); ++point)
  {
    Eigen::Vector3d delta_point = point->getVector3fMap().cast<double>() - line.point();
    double dist = delta_point.dot(line.direction());
    if (dist < min_dist)
    {
      min_proj = line.point() + (dist - 0.0)*line.direction();
      min_dist = dist;
    }
    if (dist > max_dist)
    {
      max_proj = line.point() + (dist + 0.0)*line.direction();
      max_dist = dist;
    }
  }

  Eigen::Matrix<double,6,1> line_segment;
  line_segment.segment<3>(0) = min_proj;
  line_segment.segment<3>(3) = max_proj;

  line_segment(0) += centroid_x_;
  line_segment(1) += centroid_y_;
  line_segment(3) += centroid_x_;
  line_segment(4) += centroid_y_;

  return line_segment;

}


} // namespace row_detection

