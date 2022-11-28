/**
 *  @file   se_k_3.cpp
 *  @author Ross Hartley, Wenzhe Tong
 *  @brief  Source file for various SE(3) functions
 *  @date   October 5th, 2022
 **/

#include "states/se_k_3.h"

#include <math.h>
#include <vector>

using namespace se_k_3;

// constructors
SEK3::SEK3(const Eigen::MatrixXd& X) : X_(X) {}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
}

SEK3::SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p,
           const Eigen::VectorXd& v) {
  X_.block(0, 0, 3, 3) = R;
  X_.block(0, 3, 3, 1) = p;
  X_.block(0, 4, 3, 1) = v;
}

// getters
Eigen::MatrixXd SEK3::get_X() { return X_; }
Eigen::MatrixXd SEK3::get_R() { return X_.block(0, 0, 3, 3); }
Eigen::MatrixXd SEK3::get_p() { return X_.block(0, 3, 3, 1); }
Eigen::MatrixXd SEK3::get_v() { return X_.block(0, 4, 3, 1); }
int SEK3::get_dim() { return X_.rows(); }

// setters
void SEK3::set_X(const Eigen::MatrixXd& X) { X_ = X; }
void SEK3::set_R(const Eigen::MatrixXd& R) { X_.block(0, 0, 3, 3) = R; }
void SEK3::set_p(const Eigen::VectorXd& p) { X_.block(0, 3, 3, 1) = p; }
void SEK3::set_v(const Eigen::MatrixXd& v) { X_.block(0, 4, 3, 1) = v; }

// operators
SEK3 SEK3::operator*(const SEK3& X) {
  SEK3 Y;
  // TODO: check size of X, if not same, throw error
  Y.set_X(this->X_ * X.X_);
  Y.set_K(this->K_);
  return Y;
}

// methods
SEK3 SEK3::inverse() {
  SEK3 Y;
  Y.set_R(this->get_R().transpose());
  Y.set_p(-this->get_R().transpose() * this->get_p());
  Y.set_v(-this->get_R().transpose() * this->get_v());
  Y.set_K(this->K_);
  return Y;
}
