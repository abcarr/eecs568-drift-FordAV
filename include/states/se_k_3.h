/**
 *  @file   se_k_3.h
 *  @author Wenzhe Tong
 *  @brief  Header file for various SE(3) functions
 *  @date   October 5th, 2022
 **/

#ifndef SE_K_3_H
#define SE_K_3_H


#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>


namespace se_k_3 {

class SEK3 {
 public:
  // Member variables
  int K_ = 5;
  Eigen::MatrixXd X_ = Eigen::MatrixXd::Identity(K_, K_);

  // Constructor
  SEK3() {};
  SEK3(const Eigen::MatrixXd& X) : X_(X) {};
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p) {};
  SEK3(const Eigen::MatrixXd& R, const Eigen::VectorXd& p, const Eigen::VectorXd& v) {};

  // Destructor
  ~SEK3() {}

  // Getters
  Eigen::MatrixXd get_X();
  Eigen::MatrixXd get_R();
  Eigen::MatrixXd get_p();
  Eigen::MatrixXd get_v();

  Eigen::MatrixXd get_p1();
  Eigen::MatrixXd get_v1();
  Eigen::MatrixXd get_aug(string key);
  vector<string> get_aug_keys();
  int get_aug_val(string key);
  int get_dim();

  // Setters
  void set_K(int K);
  void set_X(const Eigen::MatrixXd& X);
  void set_R(const Eigen::MatrixXd& R);
  void set_p(const Eigen::VectorXd& p);
  void set_v(const Eigen::MatrixXd& v);

  // Setters - aug state
  void set_p1(const Eigen::VectorXd& p1);
  void set_v1(const Eigen::VectorXd& v1);
  void set_aug(string key, const Eigen::VectorXd& aug);

  // Operators
  SEK3 operator*(const SEK3& X);
  SEK3 operator*(const Eigen::MatrixXd& R);
  SEK3 operator*(const Eigen::MatrixXd& p);
  SEK3 operator*(const Eigen::VectorXd& v);
  // TODO: divide operator
  void operator<<(const SEK3& X);
  void operator<<(const Eigen::MatrixXd& R);
  void operator<<(const Eigen::MatrixXd& p);
  void operator<<(const Eigen::VectorXd& v);

  // Methods
  SEK3 inverse();
  SEK3 log();
  SEK3 exp();
  SEK3 Adjoint();

  private:
    std::map<std::string, int> map_aug_;

};    // class SEK3

}    // namespace se_k_3


#endif    // SE_3_H