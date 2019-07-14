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

  QLength wheelDiam = 4.125_in;
  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withDimensions({{wheelDiam, 11.375_in}, gearsetToTPR(MOTOR_GEARSET)})
                 .build();

  // Move one wheel rotation
  drive->moveDistance(wheelDiam * 1_pi);

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
                 .withDimensions({{4.125_in, 11.375_in}, gearsetToTPR(MOTOR_GEARSET)})
                 .build();

  // Turn one wheel rotation
  drive->turnAngle(90_deg);

  double turnTicks = (11.375 / 4.125) * (90 / 360) * gearsetToTPR(MOTOR_GEARSET);
  test("Left sensor values should equal " + std::to_string(turnTicks),
       TEST_BODY(AssertThat, drive->model().getSensorVals()[0], EqualsWithDelta(turnTicks, 10)));

  test("Right sensor values should equal " + std::to_string(-turnTicks),
       TEST_BODY(AssertThat, drive->model().getSensorVals()[1], EqualsWithDelta(-turnTicks, 10)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

void runChassisControllerIntegratedTests() {
  test_printf("Testing ChassisControllerIntegrated");
  testMoveDistanceGoesTheRightDistance();
  testTurnAngleGoesTheRightDistance();
}
