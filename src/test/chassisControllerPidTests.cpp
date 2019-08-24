/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/chassisControllerPidTests.hpp"
#include "main.h"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

static void testWaitUntilSettledExitsProperly() {
  printf("Testing waitUntilSettled() exits properly\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withGains({}, {0.003})
                 .withDimensions({{4_in, 10_in}, gearsetToTPR(MOTOR_GEARSET)})
                 .build();

  for (int i = 0; i < 10; ++i) {
    drive->turnAngle(45_deg);
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

    test("Iteration " + std::to_string(i + 1), TEST_BODY(AssertThat, true, Equals(true)));
  }

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testMoveDistanceGoesTheRightDistance() {
  printf("Testing moveDistance() moves the right distance\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withGains({0.003, 0.0001, 0.00015}, {})
                 .withDimensions({{4_in, 10_in}, gearsetToTPR(MOTOR_GEARSET)})
                 .build();

  // Move one wheel rotation
  drive->moveDistance(4_in * 1_pi);

  test("Left sensor values should equal " + std::to_string(gearsetToTPR(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->model().getSensorVals()[0],
                 EqualsWithDelta(gearsetToTPR(MOTOR_GEARSET), 10)));

  test("Right sensor values should equal " + std::to_string(gearsetToTPR(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->model().getSensorVals()[1],
                 EqualsWithDelta(gearsetToTPR(MOTOR_GEARSET), 10)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testTurnAngleGoesTheRightDistance() {
  printf("Testing turnAngle() moves the right distance\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withGains({}, {0.003, 0.0001, 0.00015})
                 .withDimensions({{4_in, 365_mm}, gearsetToTPR(MOTOR_GEARSET)})
                 .build();

  // Turn one wheel rotation
  drive->turnAngle(90_deg);

  test("Left sensor values should equal " + std::to_string(gearsetToTPR(MOTOR_GEARSET)),
       TEST_BODY(AssertThat, drive->model().getSensorVals()[0], EqualsWithDelta(323, 10)));

  test("Right sensor values should equal " + std::to_string(gearsetToTPR(MOTOR_GEARSET)),
       TEST_BODY(AssertThat, drive->model().getSensorVals()[1], EqualsWithDelta(-323, 10)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

void runChassisControllerPidTests() {
  test_printf("Testing ChassisControllerPID");
  testWaitUntilSettledExitsProperly();
  testMoveDistanceGoesTheRightDistance();
  testTurnAngleGoesTheRightDistance();
}
