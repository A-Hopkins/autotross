/**
 * @file airspeed_msg.h
 * @brief Defines the airspeed message
 *
 * This file provides the definition for an airspeed sensor reading.
 */

#pragma once

#include "common_types/header.h"
#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents data from an airspeed sensor.
   *
   * This message includes differential pressure readings, calculated airspeed,
   * and supporting environmental context such as temperature and density.
   */
  DECLARE_MESSAGE_TYPE(AirspeedDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID information.
     */
    Header header;

    /**
     * @brief Differential pressure [Pascals].
     * The pressure difference measured between dynamic (pitot) and static ports.
     */
    double differential_pressure;

    /**
     * @brief Air temperature [Kelvin].
     * May represent either ambient or probe-measured temperature.
     */
    double temperature;

    /**
     * @brief True airspeed [m/s], computed from diff pressure and air density.
     */
    double true_airspeed;

    /**
     * @brief Covariance matrix (3x3) for [differential_pressure, temperature, true_airspeed].
     *
     * The rows/columns correspond to:
     * [ diff_pressure, temperature, true_airspeed ]
     *
     */
    linalg::Matrix<3, 3> covariance;

    std::string str() const
    {
      return "AirspeedDataMsg { diff_pressure: " + std::to_string(differential_pressure) +
             ", temperature: " + std::to_string(temperature) +
             ", true_airspeed: " + std::to_string(true_airspeed) +
             ", covariance: " + covariance.str() + " }";
    }
  };
} // namespace msg
