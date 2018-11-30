
#ifndef LINE_UTILS_H_
#define LINE_UTILS_H_

// XXX Includes for header only files.

namespace vineyard_row_detection
{

// Convert a line segment to polar coordinates in 2D only.
inline Eigen::Vector2f segmentToPolar(const Eigen::Matrix<float, 6, 1> & line_segment)
{

  // Get the start and end of the line in 2D.
  Eigen::Vector2f line_start_2d = line_segment.segment<2>(0);
  Eigen::Vector2f line_end_2d   = line_segment.segment<2>(3);

  float x1 = line_start_2d(0);
  float y1 = line_start_2d(1);
  float x2 = line_end_2d(0);
  float y2 = line_end_2d(1);

  // Calculate the radius of the polar coordinate.
  float length = (line_start_2d - line_end_2d).norm();
  float radius = (x1*(y1 - y2) + y1*(x2 - x1))/length;

  // Calculate the angle of the polar coordinate.
  float angle = atan2(x2 - x1, y1 - y2);

  return Eigen::Vector2f(radius, angle);

}

// Calculate the new polar coordinates of a line defined in polar coordinates due to a cartesian shift.
inline Eigen::Vector2f shiftPolarOrigin(const Eigen::Vector2f & polar, const Eigen::Vector2f & new_origin)
{

  float radius = polar(0);
  float angle  = polar(1);

  float new_radius = radius - new_origin(0)*cos(angle) - new_origin(1)*sin(angle);
  float new_angle  = angle;

  return Eigen::Vector2f(new_radius, new_angle);

}

inline Eigen::VectorXf makeRadiusPositive(const Eigen::Vector2f & polar)
{

  float radius = polar(0);
  float angle  = polar(1);

  if (radius < 0.0)
  {
    radius *= -1.0;
    angle += M_PI;
  }

  return Eigen::Vector2f(radius, angle);

}

// Normalise an angle.
inline float normaliseAngle(float angle)
{

  while (angle >= M_PI)
  {
    angle -= 2.0*M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0*M_PI;
  }

  return angle;

}

inline Eigen::Vector2f projectPointOntoPolarLine(const Eigen::Vector2f & polar_line, const Eigen::Vector2f & point)
{

  // Convert to local polar coordinates.
  Eigen::Vector2f point_local = shiftPolarOrigin(polar_line, point);

  float proj_x = point(0) + point_local(0)*cos(point_local(1));
  float proj_y = point(1) + point_local(0)*sin(point_local(1));

  return Eigen::Vector2f(proj_x, proj_y);

}

inline double yawFromQuat(const Eigen::Quaterniond & quat)
{

  double qw = quat.w();
  double qz = quat.z();

  return 2*acos(qw/sqrt(qw*qw + qz*qz));

}

} // namespace vineyard_row_detection

#endif // # ifndef LINE_UTILS_H_

