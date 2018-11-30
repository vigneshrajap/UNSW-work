
#ifndef LINE_H_
#define LINE_H_

#include <Eigen/Dense>

namespace row_detection
{

class Line
{

public:

  Line(const Eigen::Matrix<double,6,1> & line_coefficients);

  Line(const Eigen::Vector2d & polar);

  double radius() const
  {
    return radius_;
  }

  double angle() const
  {
    return angle_;
  }

  double & angle()
  {
    return angle_;
  }

  Eigen::Vector2d polar() const
  {
    return Eigen::Vector2d(radius_, angle_);
  }

  Eigen::Vector3d point() const
  {
    return point_;
  }

  Eigen::Vector3d direction() const
  {
    return direction_;
  }

  Eigen::Matrix<float,6,1> vector6fMap() const
  {
    Eigen::Matrix<float,6,1> vector_6f_map;
    vector_6f_map.segment<3>(0) = point_.cast<float>();
    vector_6f_map.segment<3>(3) = direction_.cast<float>();
    return vector_6f_map;
  }

  std::string str() const
  {
    std::stringstream ss;
    ss << point_(0) << ' ' << point_(1) << ' ' << point_(2) << ' ' << direction_(0) << ' ' << direction_(1) << ' ' << direction_(2);
    return ss.str();
  }

  void shiftOrigin(double origin_x, double origin_y)
  {
    radius_ -= origin_x*cos(angle_) + origin_y*sin(angle_);
    point_(0) -= origin_x;
    point_(1) -= origin_y;
  }

  void makeRadiusPositive();

  void normaliseAngle();

private:

  Eigen::Vector3d point_;

  Eigen::Vector3d direction_;

  double radius_;

  double angle_;

};

} // namespace row_detection

#endif // #ifndef VITI_LINE_H_

