
#ifndef LINE_DETECTION_BASE_H_
#define LINE_DETECTION_BASE_H_

#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <row_detection/line.h>

namespace row_detection
{

typedef pcl::PointXYZ point_t;
typedef pcl::PointCloud<point_t> cloud_t;
typedef cloud_t::iterator cloud_it_t;
typedef cloud_t::const_iterator cloud_const_it_t;
typedef cloud_t::Ptr cloud_ptr_t;
typedef cloud_t::ConstPtr cloud_const_ptr_t;

typedef std::vector<Line> line_vec_t;
typedef line_vec_t::iterator line_vec_it_t;

typedef Eigen::Matrix<double,6,1> line_segment_t;
typedef std::vector<line_segment_t> line_segment_vec_t;
typedef line_segment_vec_t::const_iterator line_segment_vec_it_t;


class LineDetectionBase
{

public:

  line_vec_t getLines() const
  {
    return lines_;
  }

  line_segment_vec_t getLineSegments() const
  {
    return line_segments_;
  }

  void setInputCloud(const cloud_ptr_t & cloud)
  {
    cloud_ = cloud;
  }

  virtual bool compute() = 0;

  bool save(const std::string & filename)
  {

    std::ofstream os(filename.c_str());

    if (!os.is_open())
    {
      std::cout << "Unable to open file: " << filename << std::endl;
      return false;
    }

    for (line_vec_it_t line = lines_.begin(); line != lines_.end(); ++line)
    {
      os << line->str() << std::endl;
    }

    os.close();

    return true;

  }

protected:

  cloud_ptr_t cloud_;

  line_vec_t lines_;

  line_segment_vec_t line_segments_;

  double centroid_x_;

  double centroid_y_;

  double centroid_z_;

};

} // namespace row_detection

#endif // #ifndef LINE_DETECTION_BASE_H_

