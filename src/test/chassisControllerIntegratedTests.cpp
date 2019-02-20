/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/chassisControllerIntegratedTests.hpp"
#include "main.h"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

static void testMoveDistanceGoesTheRightDistance() {
  printf("Testing moveDistance() moves the right distance\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withDimensions({{4_in, 10_in}, toUnderlyingType(MOTOR_GEARSET)})
                 .build();

  // Move one wheel rotation
  drive->moveDistance(4_in * 1_pi);

  test("Left sensor values should equal " + std::to_string(toUnderlyingType(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->getSensorVals()[0],
                 EqualsWithDelta(toUnderlyingType(MOTOR_GEARSET), 10)));

  test("Right sensor values should equal " + std::to_string(toUnderlyingType(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->getSensorVals()[1],
                 EqualsWithDelta(toUnderlyingType(MOTOR_GEARSET), 10)));

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
                 .withDimensions({{4_in, 10_in}, toUnderlyingType(MOTOR_GEARSET)})
                 .build();

  // Turn one wheel rotation
  drive->turnAngle(360 * (4.0 / 10.0) * degree);

  test("Left sensor values should equal " + std::to_string(toUnderlyingType(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->getSensorVals()[0],
                 EqualsWithDelta(toUnderlyingType(MOTOR_GEARSET), 10)));

  test("Right sensor values should equal " + std::to_string(toUnderlyingType(MOTOR_GEARSET)),
       TEST_BODY(AssertThat,
                 drive->getSensorVals()[1],
                 EqualsWithDelta(-1 * toUnderlyingType(MOTOR_GEARSET), 10)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

void runChassisControllerIntegratedTests() {
  test_printf("Testing ChassisControllerIntegrated");
  testMoveDistanceGoesTheRightDistance();
  testTurnAngleGoesTheRightDistance();
}
