/**
 * @file state_estimate_msg.h
 * @brief Defines the message for the UAV's estimated state.
 *
 * This file provides the definition for the complete state estimate produced
 * by the localization system, including position, velocity, orientation,
 * angular rates, and their associated uncertainties.
 */
#pragma once

#include "common_types/header.h"
#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents the full estimated state of the UAV from the EKF.
   *
   * This message contains the 10-dimensional state vector and the corresponding
   * 10x10 covariance matrix, providing a complete picture of the UAV's
   * estimated position, velocity, and, orientation along with
   * their uncertainties. This is the primary output of the StateEstimationTask.
   */
  DECLARE_MESSAGE_TYPE(StateEstimateMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID information.
     * The timestamp corresponds to the time of the state estimate.
     */
    Header header;

    /**
     * @brief Estimated position (x, y, z) in meters in the ENU frame.
     */
    linalg::Vector<3> position;

    /**
     * @brief Estimated velocity (vx, vy, vz) in m/s in the ENU frame.
     */
    linalg::Vector<3> velocity;

    /**
     * @brief Estimated orientation as a unit quaternion (qx, qy, qz, qw).
     * Represents the rotation from the ENU frame to the body frame.
     */
    linalg::Vector<4> orientation;

    /**
     * @brief Full 10x10 covariance matrix of the state estimate.
     * The state vector order is [pos, vel, quat].
     */
    linalg::Matrix<10, 10> covariance;

    std::string str() const
    {
      return "StateEstimateMsg { position: " + position.str() + ", velocity: " + velocity.str() + ", orientation: " + orientation.str() +
             ", covariance: " + covariance.str() + " }";
    }
  };
} // namespace msg
