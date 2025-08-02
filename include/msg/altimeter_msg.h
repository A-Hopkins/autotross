/**
 * @file altimeter_msg.h
 * @brief Defines the altimeter message
 *
 * This file provides the definition for an altimeter sensor reading.
 */
#pragma once
#include "common_types/header.h"
#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents data from a altimeter receiver.
   *
   * This message includes barometric altitude, pressure, and temperature readings
   */
  DECLARE_MESSAGE_TYPE(AltimeterDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID information.
     */
    Header header;

    /**
     * @brief Vertical position [meters].
     * Represents the current barometric altitude relative to the vertical reference.
     */
    double vertical_position;

    /**
     * @brief Vertical velocity [m/s].
     * Positive = ascending, negative = descending.
     */
    double vertical_velocity;

    /**
     * @brief Vertical reference [meters].
     * Typically corresponds to sea level, home altitude, or takeoff baseline.
     */
    double vertical_reference;

    /**
     * @brief Covariance matrix (3x3) associated with the measurement state.
     * The state vector is [vertical_position, vertical_velocity, vertical_reference].
     */
    linalg::Matrix<3, 3> covariance;

    std::string str() const
    {
      return "AltimeterDataMsg { vertical_position: " + std::to_string(vertical_position) +
             ", vertical_velocity: " + std::to_string(vertical_velocity) +
             ", vertical_reference: " + std::to_string(vertical_reference) + 
             ", covariance: " + covariance.str() + " }";
    }
  };
}
