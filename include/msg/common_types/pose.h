/**
 * @file pose.h
 * @brief Defines structures for representing 3D pose and pose with covariance.
 */
#pragma once

#include "kfplusplus/include/linalg.h"

/**
 * @brief Represents a 3D pose consisting of position and orientation.
 */
struct Pose
{
  /**
   * @brief Position in 3D space (x, y, z).
   */
  linalg::Vector<3> point;

  /**
   * @brief Orientation represented as a quaternion (x, y, z, w).
   * The w component is the scalar part.
   */
  linalg::Vector<4> orientation;

  std::string str() const
  {
    return "Pose { point: " + point.str() + ", orientation: " + orientation.str() + " }";
  }
};

/**
 * @brief Represents a 3D pose with its associated covariance matrix.
 */
struct PoseWithCovariance
{
  /**
   * @brief The pose (position and orientation).
   */
  Pose pose;

  /**
   * @brief Row-major 6x6 covariance matrix associated with the pose.
   * The state vector corresponding to this covariance matrix is
   * (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis).
   * The orientation part uses a fixed-axis representation (roll, pitch, yaw).
   */
  linalg::Matrix<6, 6> covariance;

  std::string str() const
  {
    return "PoseWithCovariance { " + pose.str() + ", covariance: " + covariance.str() + " }";
  }
};
