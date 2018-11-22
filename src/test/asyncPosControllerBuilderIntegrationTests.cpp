/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncPosControllerBuilderIntegrationTests.hpp"
#include "okapi/impl/control/async/asyncPosControllerBuilder.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

// segfault at 0301d462
void testMaxVelOnPIDController() {
  printf("Testing PID controller obeys maximum velocity\n");
  resetHardware();

  const double maxVel = 20;
  auto controller = AsyncPosControllerBuilder()
                      .withMaxVelocity(maxVel)
                      .withMotor(MOTOR_1_PORT)
                      .withGains({0.01, 0, 0})
                      .build();

  Motor motor(MOTOR_1_PORT);
  double maxRPM = 0;

  controller->setTarget(500);
  while (!controller->isSettled()) {
    const auto sampleRPM = motor.getActualVelocity();
    if (sampleRPM > maxRPM) {
      maxRPM = sampleRPM;
    }

    pros::delay(10);
  }

  controller->flipDisable(true);
  motor.moveVelocity(0);

  test("Maximum recorded RPM should be no greater than the set max velocity",
       TEST_BODY(AssertThat, maxRPM, EqualsWithDelta(maxVel, 10)));

  resetHardware();
  pros::delay(500);
}

void testMaxVelOnIntegratedController() {
  printf("Testing integrated controller obeys maximum velocity\n");
  resetHardware();

  const double maxVel = 20;
  auto controller =
    AsyncPosControllerBuilder().withMaxVelocity(maxVel).withMotor(MOTOR_1_PORT).build();

  Motor motor(MOTOR_1_PORT);
  double maxRPM = 0;

  controller->setTarget(500);
  while (!controller->isSettled()) {
    const auto sampleRPM = motor.getActualVelocity();
    if (sampleRPM > maxRPM) {
      maxRPM = sampleRPM;
    }

    pros::delay(10);
  }

  controller->flipDisable(true);
  motor.moveVelocity(0);

  test("Maximum recorded RPM should be no greater than the set max velocity",
       TEST_BODY(AssertThat, maxRPM, EqualsWithDelta(maxVel, 10)));

  resetHardware();
  pros::delay(500);
}

void runAsyncPosControllerBuilderIntegrationTests() {
  testMaxVelOnPIDController();
  testMaxVelOnIntegratedController();
}
