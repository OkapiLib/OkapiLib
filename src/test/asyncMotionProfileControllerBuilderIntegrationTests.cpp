/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncMotionProfileControllerBuilderIntegrationTests.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

static void testMotionProfileController() {
  printf("Testing motion profile controller\n");
  resetHardware();

  const auto imaginaryDiameter = 4_in;
  const auto pathLength = 10_in;

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withDimensions({{imaginaryDiameter, 10_in}, quadEncoderTPR})
                 .build();

  auto controller = AsyncMotionProfileControllerBuilder()
                      .withOutput(drive)
                      .withLimits({0.15, 2, 10})
                      .buildMotionProfileController();

  controller->generatePath({{0_in, 0_in, 0_deg}, {pathLength, 0_in, 0_deg}}, "A");

  Motor leftMtr(MOTOR_1_PORT);
  Motor rightMtr(MOTOR_2_PORT);

  controller->setTarget("A");

  AverageFilter<10> leftFilter;
  AverageFilter<10> rightFilter;
  while (!controller->isSettled()) {
    leftFilter.filter(leftMtr.getActualVelocity());
    rightFilter.filter(rightMtr.getActualVelocity());
    pros::delay(10);
  }

  controller->flipDisable(true);

  test("Steady-state average RPM should match the profile limits (left)",
       TEST_BODY(AssertThat, leftFilter.getOutput(), EqualsWithDelta(15, 2)));
  test("Steady-state average RPM should match the profile limits (right)",
       TEST_BODY(AssertThat, rightFilter.getOutput(), EqualsWithDelta(15, 2)));

  const auto endAngle = leftMtr.getPosition() * degree;
  const auto endPosition = (endAngle / 360_deg) * 1_pi * imaginaryDiameter;

  test(
    "Stopping position should roughly equal the path length",
    TEST_BODY(AssertThat, pathLength.convert(inch), EqualsWithDelta(endPosition.convert(inch), 2)));

  resetHardware();
  pros::delay(500);
}

static void testLinearMotionProfileController() {
  printf("Testing linear motion profile controller\n");
  resetHardware();

  const auto imaginaryDiameter = 1_in;
  const auto pathLength = 12_in;
  auto controller = AsyncMotionProfileControllerBuilder()
                      .withOutput(MOTOR_1_PORT, imaginaryDiameter, MOTOR_GEARSET)
                      .withLimits({0.15, 2, 10})
                      .buildLinearMotionProfileController();

  controller->moveTo(0_in, pathLength);
  controller->flipDisable(true);

  const auto endAngle = Motor(MOTOR_1_PORT).getPosition() * degree;
  const auto endPosition = (endAngle / 360_deg) * 1_pi * imaginaryDiameter;

  test(
    "Stopping position should roughly equal the path length",
    TEST_BODY(AssertThat, pathLength.convert(inch), EqualsWithDelta(endPosition.convert(inch), 2)));

  resetHardware();
  pros::delay(500);
}

void runAsyncMotionProfileControllerBuilderIntegrationTests() {
  test_printf("Testing AsyncMotionProfileControllerBuilder");
  testMotionProfileController();
  testLinearMotionProfileController();
}
