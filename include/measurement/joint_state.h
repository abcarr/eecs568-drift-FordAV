/**
 *  @file   joint_state.h
 *  @author Justin Yu
 *  @brief  Header file for robot joint state measurement
 *  @date   Nov 16, 2022
 **/

#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "measurement.h"

/**
 * @class JointStateMeasurement
 *
 * Derived measurement class containing joint
 * actuator measurements (encoder position, actuator angular velocity, acutator
 * torque).
 */
template<unsigned int JOINT_DIM, typename T>
class JointStateMeasurement : public Measurement {
 public:
  /**
   * @brief Default constructor.
   */
  JointStateMeasurement();

  /**
   * @brief Set the joint state coefficients.
   *
   * @param[in] position: vector of joint position coefficients (rad).
   * @param[in] velocity: vector of joint velocity coefficients (rad/s).
   * @param[in] effort: vector of joint effort coefficients (Newton-meters).
   */
  void set_joint_state(const Eigen::Matrix<T, JOINT_DIM, 1>& position,
                       const Eigen::Matrix<T, JOINT_DIM, 1>& velocity,
                       const Eigen::Matrix<T, JOINT_DIM, 1>& effort);

  /**
   * @brief Get the joint-axis position coefficients.
   *
   * @return Vector of joint position coefficients (rad).
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_pos() const;

  /**
   * @brief Get the joint-axis velocity coefficients.
   *
   * @return Vector of joint velocity coefficients (rad/s).
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_vel() const;

  /**
   * @brief Get the joint-axis effort (torque) coefficients.
   *
   * @return Vector of joint effort coefficients (Newton-meters).
   */
  Eigen::Matrix<T, JOINT_DIM, 1> get_joint_effort() const;


 private:
  Eigen::Matrix<T, JOINT_DIM, 1> joint_position_;
  Eigen::Matrix<T, JOINT_DIM, 1> joint_velocity_;
  Eigen::Matrix<T, JOINT_DIM, 1> joint_effort_;
};
#include "measurement/impl/joint_state_impl.cpp"

#endif
