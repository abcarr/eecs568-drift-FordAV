/* ----------------------------------------------------------------------------
 * Copyright 2024, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gps_correction.cpp
 *  @author Matthew Drosos, Alex Carr, Vidya Marri
 *  @brief  Source file for Invariant EKF gps correction method
 *  @date   April 7, 2024
 **/

#include "drift/filter/inekf/correction/gps_correction.h"

using namespace std;
using namespace math::lie_group;

namespace filter::inekf {

GpsCorrection::GpsCorrection(
    GPSQueuePtr sensorDataBufferPtr,
    std::shared_ptr<std::mutex> sensorDataBufferMutexPtr,
    const ErrorType& errorType,
    const string& yamlFilepath):
    Correction::Correction(sensorDataBufferMutexPtr),
    mSensorDataBufferPtr(sensorDataBufferPtr),
    mErrorType(errorType)
{
    correction_type_ = CorrectionType::GPS;

    cout << "Loading gps correction config from " << yamlFilepath << endl;
    YAML::Node mConfig = YAML::LoadFile(yamlFilepath);

    // set noises
    double stdev = mConfig["noises"]["gps_std"] ? mConfig["noises"]["gps_std"].as<double>(): 0.1;
    mCovariance = stdev * stdev * Eigen::Matrix<double, 3, 3>::Identity();

    // set thresholds
    t_diff_thres_ = mConfig["settings"]["correction_time_threshold"] ?
                      mConfig["settings"]["correction_time_threshold"].as<double>(): 0.3;

    // set rotation matrix from gps to body frame
    const std::vector<double> quat_gps2body
        = mConfig["settings"]["rotation_gps2body"]
            ? mConfig["settings"]["rotation_gps2body"].as<std::vector<double>>()
            : std::vector<double>({1, 0, 0, 0});

    mGpsScale = mConfig["settings"]["gps_scale"]
               ? mConfig["settings"]["gps_scale"].as<double>()
               : 1.0;

    // Convert quaternion to rotation matrix for frame transformation
    Eigen::Quaternion<double> quarternion_gps2body(
        quat_gps2body[0], quat_gps2body[1], quat_gps2body[2], quat_gps2body[3]);
        mR_gps2body = quarternion_gps2body.toRotationMatrix();
    std::string estGpsFile
        = "vanila_est_gps_log.txt";
    mEstGpsOutfile.open(estGpsFile);
    mEstGpsOutfile.precision(dbl::max_digits10);
}

GpsCorrection::~GpsCorrection() { mEstGpsOutfile.close(); }

const GPSQueuePtr GpsCorrection::get_sensor_data_buffer_ptr() const {
  return mSensorDataBufferPtr;
}

// Correct using measured body gps with the estimated gps
bool GpsCorrection::Correct(RobotState& state)
{
    Eigen::VectorXd Z;
    Eigen::MatrixXd H, N;
    int dimP = state.dimP();

    // Get latest measurement:
    sensor_data_buffer_mutex_ptr_->lock();
    if (mSensorDataBufferPtr->empty()) {
        sensor_data_buffer_mutex_ptr_->unlock();
        return false;
    }

    GPSMeasurementPtr measured_gps = mSensorDataBufferPtr->front();
    double timeDiff = measured_gps->get_time() - state.get_propagate_time();
    // only use previous gps meas to correct the state
    if (timeDiff >= 0)
    {
        sensor_data_buffer_mutex_ptr_->unlock();
        return false;
    }

    mSensorDataBufferPtr->pop();
    sensor_data_buffer_mutex_ptr_->unlock();

    if (timeDiff < -t_diff_thres_) {
        while (timeDiff < -t_diff_thres_) {
            sensor_data_buffer_mutex_ptr_->lock();
            if (mSensorDataBufferPtr->empty()) {
                sensor_data_buffer_mutex_ptr_->unlock();
                return false;
            }
            measured_gps = mSensorDataBufferPtr->front();
            mSensorDataBufferPtr->pop();
            sensor_data_buffer_mutex_ptr_->unlock();

            timeDiff = measured_gps->get_time() - state.get_propagate_time();
        }
    }

    state.set_time(measured_gps->get_time());

    // Fill out H
    H.conservativeResize(3, dimP);
    H.block(0, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
    H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

    // Fill out N
    N.conservativeResize(3, 3);
    N = state.get_world_rotation()*mCovariance*state.get_world_rotation().transpose();

    // Fill out Z
    Eigen::Matrix3d R = state.get_rotation();
    Eigen::Vector3d gps = Eigen::Vector3d(measured_gps->get_coordinates()(0),
                                          measured_gps->get_coordinates()(1),
                                          0);
    int startIndex = Z.rows();
    Z.conservativeResize(startIndex + 3, Eigen::NoChange);
    // Rotate the gps from sensor frame to body frame, then to world frame
    Z.segment(0, 3) = R * mR_gps2body * mGpsScale * gps;//measured_gps->get_coordinates();// - gps;

    if (Z.rows() > 0) {
        CorrectRightInvariant(Z, H, N, state, mErrorType);
    }

    gps = (R * mR_gps2body).inverse() * gps;//measured_gps->get_coordinates();

    mEstGpsOutfile << measured_gps->get_time() << "," << gps(0) << ","
                 << gps(1) << "," << gps(2) << std::endl
                 << std::flush;
    return true;
}

bool GpsCorrection::initialize(RobotState& state) {
    // Get measurement from sensor data buffer
    // Do not initialize if the buffer is emptys
    while (mSensorDataBufferPtr->empty()) {
        // std::cout << "Waiting for gps related encoder data..." << std::endl;
        return false;
    }

    sensor_data_buffer_mutex_ptr_.get()->lock();
    // Get the latest measurement
    while (mSensorDataBufferPtr->size() > 1) {
        // std::cout << "Discarding old sensor data..." << std::endl;
        mSensorDataBufferPtr->pop();
    }
    GPSMeasurementPtr measured_gps = mSensorDataBufferPtr->front();
    mSensorDataBufferPtr->pop();
    sensor_data_buffer_mutex_ptr_.get()->unlock();

    // Apply the rotation to map the body gps to the world frame
    Eigen::Vector3d gps = Eigen::Vector3d(measured_gps->get_coordinates()(0),
                                          measured_gps->get_coordinates()(1),
                                          0);
    state.set_position(state.get_rotation() * gps);//measured_gps->get_coordinates());
    state.set_time(measured_gps->get_time());
    return true;
}

void GpsCorrection::clear() {
    sensor_data_buffer_mutex_ptr_->lock();
    while (!mSensorDataBufferPtr->empty()) {
        mSensorDataBufferPtr->pop();
    }
    sensor_data_buffer_mutex_ptr_->unlock();
}


} // namespace filter::inekf