
#ifndef VITI_LINE_H_
#define VITI_LINE_H_

#include <Eigen/Dense>

#include <vineyard_row_detection/line_utils.h>

namespace vineyard_row_detection
{

class Line
{

public:

  Line(const Eigen::VectorXd & line_coefficients);

  Line(const Eigen::Vector2d & polar);

  double radius() const
  {
    return radius_;
  }

  double angle() const
  {
    return angle_;
  }

  Eigen::Vector2d polar() const
  {
    return Eigen::Vector2d(radius_, angle_);
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

} // namespace vineyard_row_detection

#endif // #ifndef VITI_LINE_H_

