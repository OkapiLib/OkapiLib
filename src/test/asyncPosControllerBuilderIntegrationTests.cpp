/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncPosControllerBuilderIntegrationTests.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

// segfault at 0301d462
static void testMaxVelOnPIDController() {
  printf("Testing PID controller obeys maximum velocity\n");
  resetHardware();

  const double maxVel = 20;
  auto controller = AsyncPosControllerBuilder()
                      .withMaxVelocity(maxVel)
                      .withMotor(MOTOR_1_PORT)
                      .withGains({0.01, 0, 0})
                      .build();

  Motor motor(MOTOR_1_PORT);

  controller->setTarget(500);

  AverageFilter<10> filter;
  while (!controller->isSettled()) {
    filter.filter(motor.getActualVelocity());
    pros::delay(10);
  }

  controller->flipDisable(true);
  motor.moveVelocity(0);

  test("Steady-state average RPM should be equal to max velocity",
       TEST_BODY(AssertThat, filter.getOutput(), EqualsWithDelta(maxVel, 2)));

  resetHardware();
  pros::delay(500);
}

static void testMaxVelOnIntegratedController() {
  printf("Testing integrated controller obeys maximum velocity\n");
  resetHardware();

  const double maxVel = 20;
  auto controller =
    AsyncPosControllerBuilder().withMaxVelocity(maxVel).withMotor(MOTOR_1_PORT).build();

  Motor motor(MOTOR_1_PORT);

  controller->setTarget(500);

  AverageFilter<10> filter;
  while (!controller->isSettled()) {
    filter.filter(motor.getActualVelocity());
    pros::delay(10);
  }

  controller->flipDisable(true);
  motor.moveVelocity(0);

  test("Steady-state average RPM should be equal to max velocity",
       TEST_BODY(AssertThat, filter.getOutput(), EqualsWithDelta(maxVel, 2)));

  resetHardware();
  pros::delay(500);
}

void runAsyncPosControllerBuilderIntegrationTests() {
  test_printf("Testing AsyncPosControllerBuilder");
  testMaxVelOnPIDController();
  testMaxVelOnIntegratedController();
}
