/**
 *  @file   odom.h
 *  @author Alexander Carr
 *  @brief  Header file for robot gps estimate measurement
 *  @date   April 7, 2024
 **/

#ifndef MEASUREMENT_GPS_H
#define MEASUREMENT_GPS_H

#include "measurement.h"

namespace measurement {
class GPSMeasurement : public Measurement {
 public:
  // Construct Encoder measurement
  GPSMeasurement() {
    type_ = GPS;
    coordinates_ = Eigen::Vector2d::Zero(); // latitude, longitude
  }

  GPSMeasurement(const uint64_t& latitude, const uint64_t& longitude,
                  const uint64_t seq_in, const double time_stamp_in,
                  const std::string frame_id_in) {
    type_ = GPS;

    coordinates_ = Eigen::Vector2d::Zero();

    set_coordinates(coordinates);    // [latitude, longitude]
    set_header(seq_in, time_stamp_in, frame_id_in);
  }

  void set_coordinates(const Eigen::Vector3d& position_in) {
    coordinates_(0) = latitude;
    coordinates_(1) = longitude;
  }  

  inline const Eigen::Matrix4d& get_coordinates() const {
    return coordinates_;
  }


 private:
  Eigen::Vector3d coordinates_;
};

}    // namespace measurement

#endif    // MEASUREMENT_GPS_H