
#include <vineyard_row_detection/ransac_line_detection.h>

namespace vineyard_row_detection
{

std::vector<Line> RANSACDetectLines(point_cloud_ptr_t & cloud, int n_lines, double dist_thresh)
{

  // Line model and RANSAC.
  line_model_ptr_t line_model = line_model_ptr_t(new line_model_t(cloud));
  ransac_ptr_t line_ransac = ransac_ptr_t(new ransac_t(line_model));

  std::vector<Line> lines;

  for (int i = 0; i < n_lines; ++i)
  {

    // Perform the RANSAC.
    line_ransac->computeModel();
    line_ransac->refineModel();

    // Get the model coefficients.
    Eigen::VectorXf line_coefficients;
    line_ransac->getModelCoefficients(line_coefficients);

    // Get the inliers to the refined model.
    std::vector<int> inliers;
    line_model->selectWithinDistance(line_coefficients, dist_thresh, inliers);

    // Add the line.
    lines.push_back(Line((Eigen::VectorXd)line_coefficients.cast<double>())); 

    // Remove the detected points from the cloud.
    removePointsFromCloud(*cloud, inliers);

    // XXX Hack to get the indices to clear properly.
    line_model->getIndices()->clear();
    line_model->setInputCloud(cloud);

  }

  return lines;

}

void removePointsFromCloud(point_cloud_t & cloud, const std::vector<int> & inliers)
{

  // Find outliers.
  std::vector<int> outliers(cloud.points.size() - inliers.size());
  std::vector<int>::const_iterator inlier = inliers.begin();

  for (int i = 0, j = 0; i < cloud.points.size(); ++i)
  {
    if (*inlier != i)
    {
      outliers.at(j) = i;
      ++j;
    }
    else
    {
      ++inlier;
    }
  }

  // Remove the points.
  point_cloud_t removed_cloud;
  pcl::copyPointCloud(cloud, outliers, removed_cloud);

  cloud = removed_cloud;

}

} // namespace vineyard_row_detection

