/**
 * @file msg_variant_types.h
 * @brief Defines project-specific message variant types and associated priorities for AutoTank.
 *
 * @details This header file defines the set of message types used within the AutoTank
 * application. It utilizes a macro-based approach (MESSAGE_VARIANT_TYPES) to generate:
 *   - A `std::variant` type named `MessageVariant` capable of holding any defined message type.
 *   - A `constexpr` array `message_priorities` mapping each message type to its priority level.
 *   - An `enum class Type` enumerating all defined message types.
 *   - A `constexpr` array `message_type_names` containing string representations of the message
 * types.
 *   - A utility function `msg_type_to_string` to convert the enum `Type` to its string name.
 *
 * **Override Mechanism:**
 * This file is intended to **override** the default `msg_variant_types.h`
 * (or a similarly named default file like `msg_variant_types_default.h`)
 * provided by the underlying `protocore` framework. The build system
 * (e.g., CMake) should be configured to prioritize this project-specific
 * version during compilation, ensuring that the AutoTank application uses its
 * custom set of messages and priorities instead of the framework defaults.
 *
 * **Adding New Message Types:**
 * To add a new message type:
 * 1. Include the header file for the new message struct (e.g., `"my_new_msg.h"`).
 * 2. Add a new line to the `MESSAGE_VARIANT_TYPES` macro list, specifying the
 *    message struct name and its desired priority (e.g., `X(MyNewMsg, 15),`).
 *    Ensure the struct is declared appropriately (often via `DECLARE_MESSAGE_TYPE`
 *    if using `protocore` conventions, though system messages might be handled differently).
 *
 * The rest of the definitions (`MessageVariant`, `message_priorities`, etc.) will
 * be automatically updated based on the modified `MESSAGE_VARIANT_TYPES` list.
 */
#pragma once

#include "imu_msg.h"
#include "msg/system_msgs.h"
#include <cstdint>
#include <string>
#include <variant>

namespace msg
{
// List all message types and their priorities here.
#ifndef MESSAGE_VARIANT_TYPES
#define MESSAGE_VARIANT_TYPES(X)                                       \
  X(StateMsg, 100),                                                    \
  X(StateAckMsg, 99),                                                  \
  X(HeartbeatMsg, 50),                                                 \
  X(HeartbeatAckMsg, 49),                                              \
  X(IMUDataMsg, 10)
#endif
// Message type declarations are provided in "system_msgs.h"
// Define the MessageVariant type using the list of message types.
#define MSG_VARIANT_TYPE(TYPE, PRIORITY) TYPE
  using MessageVariant = std::variant<MESSAGE_VARIANT_TYPES(MSG_VARIANT_TYPE)>;
  // Define the Type enum using the list of message types.
  enum class Type
  {
    MESSAGE_VARIANT_TYPES(MSG_VARIANT_TYPE)
  };
#undef MSG_VARIANT_TYPE

// Then, generate a compile-time array of priorities corresponding to each variant alternative:
#define MSG_VARIANT_TYPE(TYPE, PRIORITY) PRIORITY
  using Priority                          = uint16_t;
  constexpr Priority message_priorities[] = {MESSAGE_VARIANT_TYPES(MSG_VARIANT_TYPE)};
#undef MSG_VARIANT_TYPE

// Generate a compile-time array of type names
#define MSG_VARIANT_TYPE(TYPE, PRIORITY) #TYPE
  inline constexpr const char* message_type_names[] = {MESSAGE_VARIANT_TYPES(MSG_VARIANT_TYPE)};
#undef MSG_VARIANT_TYPE

  // Utility function to convert message types to a string.
  inline std::string msg_type_to_string(Type type)
  {
    size_t index = static_cast<size_t>(type);
    if (index < std::size(message_type_names))
    {
      return message_type_names[index];
    }
    return "Unknown";
  }
} // namespace msg
