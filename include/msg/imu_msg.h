/**
 * @file imu_msg.h
 * @brief Defines the imu message
 *
 * This file provides the definition for an IMU sensor reading.
 */

#pragma once

#include "common_types/header.h"
#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents data from an Inertial Measurement Unit (IMU).
   *
   * This message contains orientation, angular velocity, and linear acceleration
   * data typically provided by an IMU sensor, along with associated covariance matrices.
   */
  DECLARE_MESSAGE_TYPE(IMUDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID information.
     */
    Header header;

    /**
     * @brief ID of the IMU sensor. 
     */
     uint16_t sensor_id;

    /**
     * @brief Orientation estimate as a quaternion (x, y, z, w).
     * The order is [x, y, z, w].
     */
    linalg::Vector<4> orientation;

    /**
     * @brief Covariance matrix for the orientation estimate.
     * Row/column order corresponds to x, y, z axes of rotation.
     * Set to [-1, ...] if orientation is unknown.
     */
    linalg::Matrix<3, 3> orientation_covariance;

    /**
     * @brief Angular velocity vector in rad/s.
     * Components are [x, y, z].
     */
    linalg::Vector<3> angular_velocity;

    /**
     * @brief Covariance matrix for the angular velocity estimate.
     * Row/column order corresponds to x, y, z axes.
     */
    linalg::Matrix<3, 3> angular_velocity_covariance;

    /**
     * @brief Linear acceleration vector in m/s^2.
     * Components are [x, y, z]. Does not include gravity.
     */
    linalg::Vector<3> linear_acceleration;

    /**
     * @brief Covariance matrix for the linear acceleration estimate.
     * Row/column order corresponds to x, y, z axes.
     */
    linalg::Matrix<3, 3> linear_acceleration_covariance;

    std::string str() const
    {
      return "IMUDataMsg { orientation: " + orientation.str() +
             ", orientation_covariance: " + orientation_covariance.str() +
             ", angular_velocity: " + angular_velocity.str() +
             ", angular_velocity_covariance: " + angular_velocity_covariance.str() +
             ", linear_acceleration: " + linear_acceleration.str() +
             ", linear_acceleration_covariance: " + linear_acceleration_covariance.str() + " }";
    }

    static bool approx_equal(const IMUDataMsg& a, const IMUDataMsg& b, double eps = 1e-6)
    {

      auto to_nanosec = [](const Timestamp& t)
      {
        return static_cast<double>(t.sec) * 1e9 + static_cast<double>(t.nsec);
      };

      const uint64_t a_time = to_nanosec(a.header.stamp);
      const uint64_t b_time = to_nanosec(b.header.stamp);

      return linalg::almost_equal(a.angular_velocity, b.angular_velocity, eps) &&
             linalg::almost_equal(a.linear_acceleration, b.linear_acceleration, eps) &&
             linalg::almost_equal(a.orientation, b.orientation, eps) &&
             a_time == b_time;
    }

  };
} // namespace msg
