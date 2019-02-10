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

#define LOG_DEBUG(msg) logger->debug([&]() { return (msg).c_str(); })
#define LOG_DEBUG_S(msg) logger->debug([&]() { return msg; })

#define LOG_INFO(msg) logger->info([&]() { return (msg).c_str(); })
#define LOG_INFO_S(msg) logger->info([&]() { return msg; })

#define LOG_WARN(msg) logger->warn([&]() { return (msg).c_str(); })
#define LOG_WARN_S(msg) logger->warn([&]() { return msg; })

#define LOG_ERROR(msg) logger->error([&]() { return (msg).c_str(); })
#define LOG_ERROR_S(msg) logger->error([&]() { return msg; })

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

  constexpr bool isDebugLevelEnabled() const noexcept {
    return toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::debug);
  }

  template <typename T> constexpr void debug(T ilazyMessage) const noexcept {
    if (isDebugLevelEnabled() && logfile && timer) {
      fprintf(logfile,
              "%ld DEBUG: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              ilazyMessage());
    }
  }

  constexpr bool isInfoLevelEnabled() const noexcept {
    return toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::info);
  }

  template <typename T> constexpr void info(T ilazyMessage) const noexcept {
    if (isInfoLevelEnabled() && logfile && timer) {
      fprintf(logfile,
              "%ld INFO: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              ilazyMessage());
    }
  }

  constexpr bool isWarnLevelEnabled() const noexcept {
    return toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::warn);
  }

  template <typename T> constexpr void warn(T ilazyMessage) const noexcept {
    if (isWarnLevelEnabled() && logfile && timer) {
      fprintf(logfile,
              "%ld WARN: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              ilazyMessage());
    }
  }

  constexpr bool isErrorLevelEnabled() const noexcept {
    return toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::error);
  }

  template <typename T> constexpr void error(T ilazyMessage) const noexcept {
    if (isErrorLevelEnabled() && logfile && timer) {
      fprintf(logfile,
              "%ld ERROR: %s\n",
              static_cast<long>(timer->millis().convert(millisecond)),
              ilazyMessage());
    }
  }

  /**
   * Closes the connection to the log file.
   */
  constexpr void close() noexcept {
    if (logfile) {
      fclose(logfile);
      logfile = nullptr;
    }
  }

  private:
  const std::unique_ptr<AbstractTimer> timer;
  FILE *logfile;
  const LogLevel logLevel;
};
} // namespace okapi
