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

  GPSMeasurement() {
    type_ = GPS;
    coordinates_ = Eigen::Vector2d::Zero();
  }

  GPSMeasurement(const double& x_pos, const double& y_pos,
                  const uint64_t seq_in, const double time_stamp_in,
                  const std::string frame_id_in) {
    type_ = GPS;

    coordinates_ = Eigen::Vector2d::Zero();

    set_coordinates(x_pos, y_pos);    // [x_pos, y_pos]
    set_header(seq_in, time_stamp_in, frame_id_in);
  }

  void set_coordinates(const double& x_pos, const double& y_pos) {
    coordinates_(0) = y_pos;
    coordinates_(1) = x_pos;
  }  

  inline const Eigen::Vector2d& get_coordinates() const {
    return coordinates_;
  }


 private:
  Eigen::Vector2d coordinates_;
};

}    // namespace measurement

#endif    // MEASUREMENT_GPS_H