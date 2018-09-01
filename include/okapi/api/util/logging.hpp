/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_LOGGING_HPP_
#define _OKAPI_LOGGING_HPP_

#include "okapi/api/util/abstractTimer.hpp"
#include <memory>

namespace okapi {
class Logger {
  public:
  enum class LogLevel { off = 0, debug = 4, info = 3, warn = 2, error = 1 };

  /**
   * Initializes the logger. If the logger is not initialized when logging methods are called,
   * nothing will be logged.
   *
   * @param itimer A timer used to get the current time for log statements.
   * @param logfileName The name of the log file to open.
   * @param level The log level. Log statements above this level will be disabled.
   */
  static void initialize(std::unique_ptr<AbstractTimer> itimer,
                         std::string_view filename,
                         LogLevel level) noexcept;

  /**
   * Initializes the logger. If the logger is not initialized when logging methods are called,
   * nothing will be logged.
   *
   * @param itimer A timer used to get the current time for log statements.
   * @param logfileName The name of the log file to open.
   * @param level The log level. Log statements above this level will be disabled.
   */
  static void
  initialize(std::unique_ptr<AbstractTimer> itimer, FILE *file, LogLevel level) noexcept;

  /**
   * Get the logger instance.
   */
  static Logger *instance() noexcept;

  /**
   * Set a new logging level. Log statements above this level will be disabled. For example, if the
   * level is set to LogLevel::warn, then LogLevel::warn and LogLevel::error will be enabled, but
   * LogLevel::info and LogLevel::debug will be disabled.
   */
  static void setLogLevel(LogLevel level) noexcept;

  void debug(std::string_view message) const noexcept;

  void info(std::string_view message) const noexcept;

  void warn(std::string_view message) const noexcept;

  void error(std::string_view message) const noexcept;

  /**
   * Closes the connection to the log file.
   */
  void close() noexcept;

  private:
  Logger();
  static Logger *s_instance;
  static std::unique_ptr<AbstractTimer> timer;
  static LogLevel logLevel;
  static FILE *logfile;
};
} // namespace okapi

#endif
