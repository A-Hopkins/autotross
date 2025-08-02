/**
 * @file gps_msg.h
 * @brief Defines the gps message
 *
 * This file provides the definition for an GPS sensor reading.
 */

#pragma once

#include "common_types/header.h"
#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents data from a GPS receiver.
   *
   * This message includes global position, velocity, and associated covariance.
   */
  DECLARE_MESSAGE_TYPE(GPSDataMsg)
  {
    Header header;

    /**
     * @brief Latitude [degrees]
     */
    double latitude;

    /**
     * @brief Longitude [degrees]
     */
    double longitude;

    /**
     * @brief Altitude [meters above ellipsoid]
     */
    double altitude;

    /**
     * @brief Position covariance matrix (3x3) in ENU frame.
     */
    linalg::Matrix<3, 3> position_covariance;

    /**
     * @brief Ground velocity vector (vx, vy, vz) in [m/s], ENU frame.
     */
    linalg::Vector<3> velocity;

    /**
     * @brief Covariance matrix for velocity (3x3) in ENU frame.
     */
    linalg::Matrix<3, 3> velocity_covariance;

    std::string str() const
    {
      return "GPSDataMsg { lat: " + std::to_string(latitude) +
             ", lon: " + std::to_string(longitude) +
             ", alt: " + std::to_string(altitude) +
             ", velocity: " + velocity.str() +
             ", position_covariance: " + position_covariance.str() +
             ", velocity_covariance: " + velocity_covariance.str() + " }";
    }
  };
} // namespace msg
