
#include <row_detection/line_detection_iterative.h>

namespace row_detection
{

bool LineDetectionIterative::compute()
{

  line_segments_.clear();

  /*
  cloud_t downsampled_cloud;
  pcl::VoxelGrid<point_t> voxel_grid;
  voxel_grid.setInputCloud(cloud_);
  voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel_grid.filter(downsampled_cloud);
  //pcl::copyPointCloud(downsampled_cloud, *cloud_);
  */

  convertPointCloudToCentroidCoordinates();

  line_model_ = line_model_ptr_t(new line_model_t(cloud_));

  generateLineSamples();

  for (int i = 0; i < n_iters_; ++i)
  {
    std::vector<bool> remove_indices;
    for (line_vec_it_t line = lines_.begin(); line != lines_.end(); ++line)
    {
      remove_indices.push_back(!fitLine(*line));
    }

    for (int i = remove_indices.size() - 1; i >= 0; --i)
    {
      if (remove_indices.at(i))
      {
        lines_.erase(lines_.begin() + i);
      }
    }
  }

//  convertPointCloudToGlobalCoordinates();

  double angle_sum = 0.0;
  for (line_vec_it_t line = lines_.begin(); line != lines_.end(); ++line)
  {
    angle_sum += line->angle();
  }

  double angle_mean = angle_sum/(double)lines_.size();
  for (line_vec_it_t line = lines_.begin(); line != lines_.end(); ++line)
  {
    *line = Line(Eigen::Vector2d(line->radius(), angle_mean));
  }

  findLineSegments();

  return true;

}

void LineDetectionIterative::convertPointCloudToCentroidCoordinates()
{

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
    point->z = 0.0;
//    point->z -= centroid_z_;
  }

}

/*
void LineDetectionIterative::convertPointCloudToGlobalCoordinates()
{

  for (cloud_it_t point = cloud_->points.begin(); point != cloud_->points.end(); ++point)
  {
    point->x += centroid_x_;
    point->y += centroid_y_;
    point->z += centroid_z_;
  }

}
*/

void LineDetectionIterative::generateLineSamples()
{

  double radius_delta = (radius_max_ - radius_min_)/(double)radius_samples_;
  double angle_delta  = (angle_max_  - angle_min_ )/(double)angle_samples_;

  for (int r = 0; r < radius_samples_; ++r)
  {
    for (int a = 0; a < angle_samples_; ++a)
    {
      lines_.push_back(Line(Eigen::Vector2d(radius_min_ + (double)r*radius_delta, angle_min_ + (double)a*angle_delta)));
    }
  }

}

bool LineDetectionIterative::fitLine(Line & line)
{

  std::vector<int> inliers;

  line_model_->selectWithinDistance(line.vector6fMap(), dist_thresh_, inliers);

  if (inliers.size() < 2)
  {
    return false;
  }

  Eigen::VectorXf optimised_coeffs;
  line_model_->optimizeModelCoefficients(inliers, line.vector6fMap(), optimised_coeffs);

  line = Line((Eigen::Matrix<double,6,1>)optimised_coeffs.cast<double>());

  return true;

}

void LineDetectionIterative::findLineSegments()
{

  for (line_vec_it_t line = lines_.begin(); line != lines_.end(); ++line)
  {
    std::vector<int> inliers;
    line_model_->selectWithinDistance(line->vector6fMap(), dist_thresh_, inliers);
    cloud_t line_cloud;
    pcl::copyPointCloud(*cloud_, inliers, line_cloud);
    line_segments_.push_back(findLineSegment(*line, line_cloud));
  }

}

Eigen::Matrix<double,6,1> LineDetectionIterative::findLineSegment(const Line & line, const cloud_t & cloud)
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
      min_proj = line.point() + (dist - 2.0)*line.direction();
      min_dist = dist;
    }
    if (dist > max_dist)
    {
      max_proj = line.point() + (dist + 2.0)*line.direction();
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

void LineDetectionIterative::saveLineSegments(const std::string & filename)
{

  std::ofstream os(filename.c_str());
  os << std::fixed;

  for (std::vector<Eigen::Matrix<double,6,1> >::iterator line_segment = line_segments_.begin(); line_segment != line_segments_.end(); ++line_segment)
  {
    for (int i = 0; i < line_segment->rows(); ++i)
    {
      os << (*line_segment)(i) << ' ';
    }
    os << std::endl;
  }

  os.close();

}

} // namespace row_detection

