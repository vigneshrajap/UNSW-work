
#include <vineyard_row_detection/line.h>

namespace vineyard_row_detection
{

Line::Line(const Eigen::VectorXd & line_coefficients)
{

  // Copy the point and direction.
  point_ = line_coefficients.segment<3>(0);
  direction_ = line_coefficients.segment<3>(3);

  double x1 = point_(0);
  double y1 = point_(1);

  double x2 = point_(0) + direction_(0);
  double y2 = point_(1) + direction_(1);

  double dx = x2 - x1;
  double dy = y2 - y1;

  double length = sqrt(dx*dx + dy*dy);

  radius_ = (-x1*dy + y1*dx)/length;
  angle_ = atan2(dx, -dy);

}

Line::Line(const Eigen::Vector2d & polar)
{

  radius_ = polar(0);
  angle_ = polar(1);

  point_(0) = radius_*cos(angle_);
  point_(1) = radius_*sin(angle_);

  direction_(0) = cos(angle_ + M_PI_2);
  direction_(1) = sin(angle_ + M_PI_2);

}

void Line::makeRadiusPositive()
{

  if (radius_ < 0.0)
  {
    radius_ *= -1.0;
    angle_ += M_PI;
  }

  normaliseAngle();

}

void Line::normaliseAngle()
{

  while (angle_ > M_PI)   { angle_ -= 2.0*M_PI; }
  while (angle_ <= -M_PI) { angle_ += 2.0*M_PI; }

}

} // namespace vineyard_row_detection

