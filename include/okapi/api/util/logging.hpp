/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <memory>

namespace okapi {
class Logger {
  public:
  enum class LogLevel { debug = 4, info = 3, warn = 2, error = 1, off = 0 };

  /**
   * A logger that does nothing.
   */
  Logger() noexcept : Logger(nullptr, nullptr, LogLevel::off) {
  }

  /**
   * A logger that opens the input file name with append permissions.
   *
   * @param itimer A timer used to get the current time for log statements.
   * @param ifileName The name of the log file to open.
   * @param ilevel The log level. Log statements more verbose than this level will be disabled.
   */
  Logger(std::unique_ptr<AbstractTimer> itimer,
         std::string_view ifileName,
         const LogLevel &ilevel) noexcept
    : Logger(std::move(itimer), fopen(ifileName.data(), "a"), ilevel) {
  }

  /**
   * A logger that uses an existing file handle. Will be closed by the logger!
   *
   * @param itimer A timer used to get the current time for log statements.
   * @param ifile The log file to open. Will be closed by the logger!
   * @param ilevel The log level. Log statements more verbose than this level will be disabled.
   */
  Logger(std::unique_ptr<AbstractTimer> itimer, FILE *const ifile, const LogLevel &ilevel) noexcept
    : timer(std::move(itimer)), logfile(ifile), logLevel(ilevel) {
  }

  ~Logger() {
    if (logfile) {
      fclose(logfile);
      logfile = nullptr;
    }
  }

  constexpr void debug(std::string_view message) const noexcept {
    if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::info) && logfile && timer) {
      fprintf(logfile,
              "%ld DEBUG: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              message.data());
    }
  }

  constexpr void info(std::string_view message) const noexcept {
    if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::info) && logfile && timer) {
      fprintf(logfile,
              "%ld INFO: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              message.data());
    }
  }

  constexpr void warn(std::string_view message) const noexcept {
    if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::warn) && logfile && timer) {
      fprintf(logfile,
              "%ld WARN: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              message.data());
    }
  }

  constexpr void error(std::string_view message) const noexcept {
    if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::error) && logfile && timer) {
      fprintf(logfile,
              "%ld ERROR: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              message.data());
    }
  }

  /**
   * Closes the connection to the log file.
   */
  void close() noexcept {
    if (logfile) {
      fclose(logfile);
      logfile = nullptr;
    }
  }

  private:
  std::unique_ptr<AbstractTimer> timer;
  FILE *logfile;
  LogLevel logLevel;
};
} // namespace okapi
