/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "main.h"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

void testWaitUntilSettledExitsProperly() {
  printf("Testing waitUntilSettled() exits properly\n");

  auto drive = ChassisControllerFactory::create(
    MOTOR_1_PORT * -1, MOTOR_2_PORT, {0.003}, {0}, {0.004}, AbstractMotor::gearset::green, {1, 1});

  for (int i = 0; i < 10; ++i) {
    drive.turnAngle(45_deg);
    for (int i = 0; i < 100; ++i) {
      int32_t mtr1Vel = pros::c::motor_get_target_velocity(MOTOR_1_PORT);
      int32_t mtr2Vel = pros::c::motor_get_target_velocity(MOTOR_2_PORT);

      if (mtr1Vel != 0) {
        test("Motor 1 vel should be zero", TEST_BODY(AssertThat, mtr1Vel, Equals(0)));
      }

      if (mtr2Vel != 0) {
        test("Motor 2 vel should be zero", TEST_BODY(AssertThat, mtr2Vel, Equals(0)));
      }

      pros::delay(10);
    }
  }

  drive.stop();
}

void runChassisControllerPidTests() {
  test_printf("Testing ChassisControllerPID");
  testWaitUntilSettledExitsProperly();
}
