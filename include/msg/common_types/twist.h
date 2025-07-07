/**
 * @file twist.h
 * @brief Defines structures for representing linear and angular velocity (twist)
 *        and twist with covariance.
 */
#pragma once

#include "kfplusplus/include/linalg.h"

/**
 * @brief Represents the twist of a body in 3D space.
 * Twist combines linear and angular velocity components.
 */
struct Twist
{
  /**
   * @brief Linear velocity component (vx, vy, vz).
   * Represents the rate of change of position.
   */
  linalg::Vector<3> linear;

  /**
   * @brief Angular velocity component (wx, wy, wz).
   * Represents the rate of change of orientation around the x, y, and z axes.
   */
  linalg::Vector<3> angular;

  std::string str() const
  {
    return "Twist { linear: " + linear.str() + ", angular: " + angular.str() + " }";
  }
};

/**
 * @brief Represents a twist with its associated uncertainty (covariance).
 */
struct TwistWithCovariance
{
  /**
   * @brief The twist data (linear and angular velocity).
   */
  Twist twist;

  /**
   * @brief Row-major 6x6 covariance matrix associated with the twist.
   * The state vector corresponding to this covariance matrix is
   * (vx, vy, vz, wx, wy, wz), representing linear velocities followed by
   * angular velocities.
   */
  linalg::Matrix<6, 6> covariance;

  std::string str() const
  {
    return "TwistWithCovariance { " + twist.str() + ", covariance: " + covariance.str() + " }";
  }
};
