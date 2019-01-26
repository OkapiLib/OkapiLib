/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ThreadedMockMotorTest : public ::testing::Test {
  public:
  void SetUp() override {
    Test::SetUp();
    motor = new ThreadedMockMotor();
    motor->startThread();
  }

  void TearDown() override {
    Test::TearDown();
    delete motor;
  }

  ThreadedMockMotor *motor;
};

TEST_F(ThreadedMockMotorTest, BasicTest) {
  motor->moveAbsolute(600, 200);
  while (motor->getPosition() < 600) {
    std::cout << motor->getPosition() << ", " << motor->getActualVelocity() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  motor->moveRelative(100, 50);
  while (motor->getPosition() < 700) {
    std::cout << motor->getPosition() << ", " << motor->getActualVelocity() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
