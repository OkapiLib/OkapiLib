/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/logging.hpp"

namespace okapi {
std::shared_ptr<Logger> defaultLogger;

int DefaultLoggerInitializer::count;

Logger::Logger() noexcept : Logger(nullptr, nullptr, LogLevel::off) {
}

Logger::Logger(std::unique_ptr<AbstractTimer> itimer,
               std::string_view ifileName,
               const Logger::LogLevel &ilevel) noexcept
  : Logger(std::move(itimer),
           fopen(ifileName.data(), isSerialStream(ifileName) ? "w" : "a"),
           ilevel) {
}

Logger::Logger(std::unique_ptr<AbstractTimer> itimer,
               FILE *const ifile,
               const Logger::LogLevel &ilevel) noexcept
  : timer(std::move(itimer)), logLevel(ilevel), logfile(ifile) {
}

Logger::~Logger() {
  if (logfile) {
    fclose(logfile);
    logfile = nullptr;
  }
}

std::shared_ptr<Logger> Logger::getDefaultLogger() {
  return defaultLogger;
}

void Logger::setDefaultLogger(std::shared_ptr<Logger> ilogger) {
  defaultLogger = std::move(ilogger);
}

bool Logger::isSerialStream(std::string_view filename) {
  return filename.find("/ser/") != std::string::npos;
}
} // namespace okapi
