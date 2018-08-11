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
    Logger::instance()->close();
    free(logBuffer);
  }

  void logData(Logger *logger) const {
    logger->error("MSG");
    logger->warn("MSG");
    logger->info("MSG");
    logger->debug("MSG");
  }

  FILE *logFile;
  char *logBuffer;
  size_t logSize;
};

TEST_F(LoggerTest, OffLevel) {
  Logger::initialize(std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::off);
  auto logger = Logger::instance();

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
  Logger::initialize(std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::error);
  auto logger = Logger::instance();

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
  Logger::initialize(std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::warn);
  auto logger = Logger::instance();

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
  Logger::initialize(std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::info);
  auto logger = Logger::instance();

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
  Logger::initialize(std::make_unique<ConstantMockTimer>(0_ms), logFile, Logger::LogLevel::debug);
  auto logger = Logger::instance();

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
