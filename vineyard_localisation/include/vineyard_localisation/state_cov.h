
#ifndef VITI_STATE_COV_H_
#define VITI_STATE_COV_H_

#include <Eigen/Dense>

namespace vineyard_localisation
{

class StateCov
{

protected:

  double tx() const { return state_(0); }
  double & tx() { return state_(0); }

  double ty() const { return state_(1); }
  double & ty() { return state_(1); }

  double tz() const { return state_(2); }
  double & tz() { return state_(2); }

  Eigen::Vector3d trans() const { return Eigen::Vector3d(tx(), ty(), tz()); }

  double qw() const { return state_(3); }
  double & qw() { return state_(3); }

  double qx() const { return state_(4); }
  double & qx() { return state_(4); }

  double qy() const { return state_(5); }
  double & qy() { return state_(5); }

  double qz() const { return state_(6); }
  double & qz() { return state_(6); }

  Eigen::Quaterniond quat() const { return Eigen::Quaterniond(qw(), qx(), qy(), qz()); }

  void normaliseQuat()
  {
    Eigen::Quaterniond rot(qw(), qx(), qy(), qz());
    rot.normalize();
    state_(3) = rot.w();
    state_(4) = rot.x();
    state_(5) = rot.y();
    state_(6) = rot.z();
  }

  Eigen::VectorXd & state() { return state_; }
  const Eigen::VectorXd & state() const { return state_; }

  Eigen::MatrixXd & cov() { return cov_; }
  const Eigen::MatrixXd & cov() const { return cov_; }

  virtual int size() { return state_.rows(); }

  Eigen::VectorXd state_;

  Eigen::MatrixXd cov_;

};

} // namespace vineyard_localisation

#endif // #ifndef VITI_STATE_COV_H_

