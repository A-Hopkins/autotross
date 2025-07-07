/**
 * @file header.h
 * @brief Standard metadata for data types
 *
 */
#pragma once
#include <cstdint>
#include <string>

namespace msg
{
  /**
   * @struct Timestamp
   * @brief Represents a time value as seconds and nanoseconds.
   */
  struct Timestamp
  {
    /** @brief Seconds component of the timestamp (since epoch). */
    uint32_t sec;
    /** @brief Nanoseconds component of the timestamp (since seconds). */
    uint32_t nsec;
  };

  /**
   * @struct Header
   * @brief Standard metadata for messages, including sequence number, timestamp, and frame ID.
   *        This is commonly used to provide context for sensor data or other messages.
   */
  struct Header
  {
    /** @brief Sequence ID: consecutively increasing identifier. */
    uint32_t seq;

    /**
     * @brief Timestamp associated with the data.
     *        Represents the time of data acquisition or generation.
     *        * stamp.sec: seconds (stamp_secs) since epoch
     *        * stamp.nsec: nanoseconds since stamp_secs
     */
    Timestamp stamp;

    /**
     * @brief Frame ID: the coordinate frame this data is associated with.
     *        Typically a string identifier like "odom", "base_link", etc.
     */
    std::string frame_id;
  };
} // namespace msg
