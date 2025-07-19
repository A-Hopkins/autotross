/**
 * @file scoped_timer.h
 * @brief Defines the inline scoped timer
 *
 * A handy utility that uses RAII scoped timing for a function
 */

#pragma once
#include "protocore/include/logger.h"

#include <chrono>
#include <iostream>

struct ScopedTimer
{
  const char*                           name;
  std::chrono::steady_clock::time_point start;

  ScopedTimer(const char* name) : name(name), start(std::chrono::steady_clock::now()) {}
  ~ScopedTimer()
  {
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - start)
                  .count();
    Logger::instance().log(LogLevel::DEBUG, name, " took " + std::to_string(dt) + " microseconds");
  }
};
