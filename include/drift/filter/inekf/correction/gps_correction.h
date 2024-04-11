/* ----------------------------------------------------------------------------
 * Copyright 2024, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gps_correction.h
 *  @author Matthew Drosos, Alex Carr, Vidya Marri
 *  @brief  Header file for Invariant EKF gps correction method
 *  @date   April 7, 2024
 **/

#ifndef FILTER_INEKF_CORRECTION_GPS_CORRECTION_H
#define FILTER_INEKF_CORRECTION_GPS_CORRECTION_H

#include <fstream>
#include <iomanip>
#include <iostream>

#include "drift/filter/base_correction.h"
#include "drift/filter/inekf/inekf.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/gps.h"
#include "drift/utils/type_def.h"

using namespace math;
using namespace state;
using namespace measurement;

namespace filter::inekf {

/**
 * @class GpsCorrection
 * @brief A class for state correction using gps measurement data.
 *
 * A class for state correction using gps measurement data. This class
 * handles the correction of the state estimate using the measured gps
 * between the body frame and the world frame. Default is a right-invariant
 * measurement model.
 *
 **/
class GpsCorrection : public Correction {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors
    /// @{
    /**
     * @brief Constructor for gps correction class.
     *
     * @param[in] sensor_data_buffer_ptr: Pointer to the buffer of sensor data
     * @param[in] sensor_data_buffer_mutex_ptr: Pointer to the mutex for the
     * sensor data buffer
     * @param[in] error_type: Error type for the correction. LeftInvariant or
     * RightInvariant
     * @param[in] yaml_filepath: Path to the yaml file for the correction
     */
    GpsCorrection(GPSQueuePtr sensorDataBufferPtr,
                  std::shared_ptr<std::mutex> sensorDataBufferMutexPtr,
                  const ErrorType& errorType,
                  const std::string& yamlFilepath);

    ~GpsCorrection();

    /// @name Correction Methods
    /// @{
    // ======================================================================
    /**
     * @brief Corrects the state estimate using measured gps [m] that
     * is measured and covarinace of the gps. Measurements are taken in
     * body frame. This is a right-invariant measurement model.
     *
     * @param[in,out] state: the current state estimate
     * @return bool: successfully correct state or not (if we do not receive a
     * new message and this method is called it'll return false.)
     */
    bool Correct(RobotState& state) override;

    /// @name Getters
    /// @{
    // ======================================================================
    /**
     * @brief Return the pointer of the sensor data buffer
     *
     * @return GPSQueuePtr: pointer of the sensor data buffer
     */
    const GPSQueuePtr get_sensor_data_buffer_ptr() const;

    /**
     * @brief Set the initial gps of the robot
     *
     * @param[in,out] state: the current state estimate, which will be initialized
     * @return bool: whether the initialization is successful
     */
    bool initialize(RobotState& state) override;

    /**
     * @brief Clear the sensor_data_buffer
     *
     */
    void clear() override;

  private:

    const ErrorType mErrorType; /**> Error type for the correction,
                               LeftInvariant or RightInvariant. */
    GPSQueuePtr
    	mSensorDataBufferPtr; /**> Pointer to the sensor buffer. */

    Eigen::Matrix3d mCovariance; /**> Gps covariance. */
    Eigen::Matrix3d mR_gps2body; /**> Rotation matrix from gps frame to body
                                  	frame. It stores the value from config file
                                  	when the class object is created.*/
    double mGpsScale;

    std::ofstream mEstGpsOutfile;
};

} // end namespace filter::inekf

#endif



