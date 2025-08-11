/**
 * @file geo_utils.h
 * @brief Utility functions for geo conversions
 */
#pragma once

#include "kfplusplus/include/linalg.h"
#include <cmath>

namespace geo_utils
{
  // WGS-84 ellipsoid parameters
  constexpr double WGS84_A    = 6378137.0;        ///< Semi major axis [m]
  constexpr double WGS84_E_SQ = 6.69437999014e-3; ///< First eccentricity squared

  /**
   * @brief Converts Geodetic coordinates (Latitude, Longitude, Altitude) to
   * Earth-Centered, Earth-Fixed (ECEF) coordinates.
   *
   * @param lat Latitude in [degrees].
   * @param lon Longitude in [degrees].
   * @param alt Altitude in [m].
   * @return A 3D vector of ECEF coordinates (x, y, z).
   */
  inline linalg::Vector<3> lla_to_ecef(double lat, double lon, double alt)
  {
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double cos_lat = std::cos(lat_rad);
    double sin_lat = std::sin(lat_rad);

    double N = WGS84_A / std::sqrt(1.0 - WGS84_E_SQ * sin_lat * sin_lat);

    linalg::Vector<3> ecef;
    ecef(0) = (N + alt) * cos_lat * std::cos(lon_rad);  // x
    ecef(1) = (N + alt) * cos_lat * std::sin(lon_rad);  // y
    ecef(2) = ((1.0 - WGS84_E_SQ) * N + alt) * sin_lat; // z
    return ecef;
  }

  /**
   * @brief Converts a point's ECEF coordinates to a local East-North-Up (ENU)
   * frame, given a reference origin in ECEF.
   *
   * @param ecef_point The ECEF coordinates of the point to convert.
   * @param ecef_ref The ECEF coordinates of the ENU frame's origin.
   * @param ref_lat The latitude of the origin (in degrees), needed for the rotation matrix.
   * @param ref_lon The longitude of the origin (in degrees), needed for the rotation matrix.
   * @return A 3D vector of the point's coordinates in the ENU frame.
   */
  inline linalg::Vector<3> ecef_to_enu(const linalg::Vector<3>& ecef_point, const linalg::Vector<3>& ecef_ref, double ref_lat,
                                       double ref_lon)
  {
    double lat_rad = ref_lat * M_PI / 180.0;
    double lon_rad = ref_lon * M_PI / 180.0;
    double cos_lat = std::cos(lat_rad);
    double sin_lat = std::sin(lat_rad);
    double cos_lon = std::cos(lon_rad);
    double sin_lon = std::sin(lon_rad);

    linalg::Vector<3> diff = ecef_point - ecef_ref;

    // Rotation matrix to transform ECEF delta to ENU
    linalg::Matrix<3, 3> R;
    R(0, 0) = -sin_lon;
    R(0, 1) = cos_lon;
    R(0, 2) = 0.0;
    R(1, 0) = -sin_lat * cos_lon;
    R(1, 1) = -sin_lat * sin_lon;
    R(1, 2) = cos_lat;
    R(2, 0) = cos_lat * cos_lon;
    R(2, 1) = cos_lat * sin_lon;
    R(2, 2) = sin_lat;

    return R * diff;
  }

} // namespace geo_utils