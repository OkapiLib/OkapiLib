/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/logging.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class LoggerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    logFile = open_memstream(&logBuffer, &logSize);
  }

  virtual void TearDown() {
    // Call close after every case so other tests don't end up with a NULL logfile pointer
    if (logger) {
      logger->close();
    }
    free(logBuffer);
  }

  void logData(const std::shared_ptr<Logger> &ilogger) const {
    ilogger->error("MSG");
    ilogger->warn("MSG");
    ilogger->info("MSG");
    ilogger->debug("MSG");
  }

  FILE *logFile;
  char *logBuffer;
  size_t logSize;
  std::shared_ptr<Logger> logger;
};

TEST_F(LoggerTest, OffLevel) {
  logger = std::make_shared<Logger>(
    std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::off);

  logData(logger);

  char *line = nullptr;
  size_t len;

  fputs("EMPTY_FILE", logFile);

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "EMPTY_FILE");

  if (line) {
    free(line);
  }
}

TEST_F(LoggerTest, ErrorLevel) {
  logger = std::make_shared<Logger>(
    std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::error);

  logData(logger);

  char *line = nullptr;
  size_t len;

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 ERROR: MSG\n");

  if (line) {
    free(line);
  }
}

TEST_F(LoggerTest, WarningLevel) {
  logger = std::make_shared<Logger>(
    std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::warn);

  logData(logger);

  char *line = nullptr;
  size_t len;

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 ERROR: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 WARN: MSG\n");

  if (line) {
    free(line);
  }
}

TEST_F(LoggerTest, InfoLevel) {
  logger = std::make_shared<Logger>(
    std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::info);

  logData(logger);

  char *line = nullptr;
  size_t len;

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 ERROR: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 WARN: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 INFO: MSG\n");

  if (line) {
    free(line);
  }
}

TEST_F(LoggerTest, DebugLevel) {
  logger = std::make_shared<Logger>(
    std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::debug);

  logData(logger);

  char *line = nullptr;
  size_t len;

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 ERROR: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 WARN: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 INFO: MSG\n");

  getline(&line, &len, logFile);
  EXPECT_STREQ(line, "0 DEBUG: MSG\n");

  if (line) {
    free(line);
  }
}
