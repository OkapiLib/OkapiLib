/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncMotionProfileControllerBuilderIntegrationTests.hpp"
#include "okapi/impl/control/async/asyncPosControllerBuilder.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

void testMotionProfileController() {
  printf("Testing motion profile controller\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withDimensions({4_in, 10_in})
                 .build();

  auto controller = AsyncMotionProfileControllerBuilder()
                      .withOutput(drive)
                      .withLimits({1, 2, 10})
                      .buildMotionProfileController();

  controller->generatePath({{0_in, 0_in, 0_deg}, {1_in, 0_in, 0_deg}}, "A");

  double maxVelLeft = 0;
  double maxVelRight = 0;
  Motor leftMtr(MOTOR_1_PORT);
  Motor rightMtr(MOTOR_2_PORT);

  controller->setTarget("A");

  while (!controller->isSettled()) {
    const auto leftVel = leftMtr.getActualVelocity();
    const auto rightVel = rightMtr.getActualVelocity();

    if (leftVel > maxVelLeft) {
      maxVelLeft = leftVel;
    }

    if (rightVel > maxVelRight) {
      maxVelRight = rightVel;
    }
  }

  controller->flipDisable(true);

  resetHardware();
  pros::delay(500);
}

void runAsyncMotionProfileControllerBuilderIntegrationTests() {
  test_printf("Testing AsyncMotionProfileControllerBuilder");
  testMotionProfileController();
}
