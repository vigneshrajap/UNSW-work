
#include <vineyard_row_detection/line_detector.h>

namespace vineyard_row_detection
{

LineDetector::LineDetector()
{

  cloud_ = point_cloud_ptr_t(new point_cloud_t());
  line_model_ = line_model_ptr_t(new line_model_t(cloud_));
  ransac_ = ransac_ptr_t(new ransac_t(line_model_));

  ransac_->setDistanceThreshold(0.5);

}

void LineDetector::detectLines(Eigen::Matrix<float, 6, 10> & lines_coefficients)
{

  for (int i = 0; i < lines_coefficients.cols(); ++i)
  {

    line_model_->getIndices()->clear();
    line_model_->setInputCloud(cloud_); 

    // Perform RANSAC.
    std::vector<int> inliers;
    ransac_->computeModel();
    ransac_->refineModel();

    // Get model coefficients.
    Eigen::VectorXf line_coefficients;
    ransac_->getModelCoefficients(line_coefficients);

    line_model_->selectWithinDistance(line_coefficients, 0.5, inliers);

    // Remove detected points.
    point_cloud_t full_cloud, line_cloud;
    pcl::copyPointCloud(*cloud_, full_cloud);
    line_cloud.points.resize(inliers.size());
    cloud_->points.resize(full_cloud.points.size() - inliers.size());

    int inlier_idx = 0, outlier_idx = 0;

    for (int j = 0; j < full_cloud.points.size(); ++j)
    {
      if (inliers[inlier_idx] == j)
      {
        line_cloud.points[inlier_idx] = full_cloud.points[j];
        ++inlier_idx;
        if (inlier_idx == inliers.size())
        {
          break;
        }
      }
      else
      {
        cloud_->points[outlier_idx] = full_cloud.points[j];
        ++outlier_idx;
      }
    }

    // Recalculate the parameters based on the start and end of the line.
    findStartAndEndPoints(line_cloud, line_coefficients);

    lines_coefficients.col(i) = line_coefficients;

  }

}

void LineDetector::findStartAndEndPoints(const point_cloud_t & line_cloud, Eigen::VectorXf & line_coefficients)
{

  Eigen::Vector3f line_point = line_coefficients.segment<3>(0);
  Eigen::Vector3f line_dir   = line_coefficients.segment<3>(3);
  line_dir.normalize();

  float min_dist = std::numeric_limits<float>::max(), max_dist = std::numeric_limits<float>::min();
  Eigen::Vector3f min_proj, max_proj;

  // Find the length of the projections onto the line, and take the maximum and minimum values.
  for (int i = 0; i < line_cloud.points.size(); ++i)
  {

    Eigen::Vector3f point(line_cloud.points[i].x, line_cloud.points[i].y, line_cloud.points[i].z);
    Eigen::Vector3f delta_point = point - line_point;

    float dist = delta_point.dot(line_dir);

    if (dist < min_dist)
    {
      min_proj = line_point + dist*line_dir;
      min_dist = dist;
    }
    else if (dist > max_dist)
    {
      max_proj = line_point + dist*line_dir;
      max_dist = dist;
    }

  }

  line_coefficients.segment<3>(0) = min_proj; 
  line_coefficients.segment<3>(3) = max_proj; 

}

} // namespace vineyard_row_detection

