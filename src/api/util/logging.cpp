/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <stdio.h>

namespace okapi {
Logger *Logger::s_instance;
std::unique_ptr<AbstractTimer> Logger::timer;
Logger::LogLevel Logger::logLevel;
FILE *Logger::logfile;

Logger::Logger() = default;

Logger *Logger::instance() noexcept {
  if (!s_instance) {
    s_instance = new Logger();
  }

  return s_instance;
}

void Logger::initialize(std::unique_ptr<AbstractTimer> itimer,
                        std::string_view filename,
                        Logger::LogLevel level) noexcept {
  timer = std::move(itimer);
  logLevel = level;

  FILE *log = fopen(filename.data(), "w");
  if (log) {
    logfile = log;
  }
}

void Logger::initialize(std::unique_ptr<AbstractTimer> itimer,
                        FILE *file,
                        Logger::LogLevel level) noexcept {
  timer = std::move(itimer);
  logLevel = level;

  if (file) {
    logfile = file;
  }
}

void Logger::setLogLevel(Logger::LogLevel level) noexcept {
  logLevel = level;
}

void Logger::debug(std::string_view message) const noexcept {
  if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::info) && logfile && timer) {
    fprintf(logfile,
            "%ld DEBUG: %s\n",
            static_cast<long>(timer->millis().convert(millisecond)),
            message.data());
  }
}

void Logger::info(std::string_view message) const noexcept {
  if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::info) && logfile && timer) {
    fprintf(logfile,
            "%ld INFO: %s\n",
            static_cast<long>(timer->millis().convert(millisecond)),
            message.data());
  }
}

void Logger::warn(std::string_view message) const noexcept {
  if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::warn) && logfile && timer) {
    fprintf(logfile,
            "%ld WARN: %s\n",
            static_cast<long>(timer->millis().convert(millisecond)),
            message.data());
  }
}

void Logger::error(std::string_view message) const noexcept {
  if (toUnderlyingType(logLevel) >= toUnderlyingType(LogLevel::error) && logfile && timer) {
    fprintf(logfile,
            "%ld ERROR: %s\n",
            static_cast<long>(timer->millis().convert(millisecond)),
            message.data());
  }
}

void Logger::close() noexcept {
  fclose(logfile);
  logfile = nullptr;
}
} // namespace okapi
