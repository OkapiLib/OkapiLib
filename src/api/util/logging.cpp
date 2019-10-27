/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/logging.hpp"

namespace okapi {
static int defaultLoggerNiftyCounter;

static typename std::aligned_storage<sizeof(Logger), alignof(Logger)>::type defaultLoggerMemBuf;

static typename std::aligned_storage<sizeof(std::shared_ptr<Logger>),
                                     alignof(std::shared_ptr<Logger>)>::type defaultLoggerBuf;

Logger &defaultLoggerMem = reinterpret_cast<Logger &>(defaultLoggerMemBuf);

std::shared_ptr<Logger> &defaultLogger =
  reinterpret_cast<std::shared_ptr<Logger> &>(defaultLoggerBuf);

DefaultLoggerInitializer::DefaultLoggerInitializer() {
  if (defaultLoggerNiftyCounter++ == 0) {
    new (&defaultLoggerMem) Logger();
    new (&defaultLogger) std::shared_ptr<Logger>(&defaultLoggerMem);
  }
}

DefaultLoggerInitializer::~DefaultLoggerInitializer() = default;

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
  new (&defaultLogger) std::shared_ptr<Logger>(std::move(ilogger));
}

bool Logger::isSerialStream(std::string_view filename) {
  return filename.find("/ser/") != std::string::npos;
}
} // namespace okapi
