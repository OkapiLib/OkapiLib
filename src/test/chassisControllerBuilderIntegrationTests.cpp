/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/chassisControllerBuilderIntegrationTests.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

void testForwardUsesCorrectMaximumVelocityForAGearset() {
  printf("Testing forward uses the correct max vel for a gearset\n");

  const double power = 0.2;
  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .build();

  drive->forward(power);
  pros::delay(500);

  auto sampleRPM = Motor(MOTOR_1_PORT).getActualVelocity();
  test(
    "Sample RPM should be approximately equal to the gearset rpm multiplied by the power",
    TEST_BODY(AssertThat, sampleRPM, EqualsWithDelta(toUnderlyingType(MOTOR_GEARSET) * power, 3)));

  drive->stop();
  pros::delay(500);
}

void testMaxVelWorksOutOfOrder() {
  printf("Testing withMaxVelocity works out of order\n");

  const double maxVel = 20;
  auto drive = ChassisControllerBuilder()
                 .withMaxVelocity(maxVel)
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .build();

  drive->forward(1);
  pros::delay(500);

  auto sampleRPM = Motor(MOTOR_1_PORT).getActualVelocity();
  test("Sample RPM should be approximately 20",
       TEST_BODY(AssertThat, sampleRPM, EqualsWithDelta(maxVel, 2)));

  drive->stop();
  pros::delay(500);
}

void runChassisControllerFactoryIntegrationTests() {
  test_printf("Testing ChassisControllerBuilder");
  testForwardUsesCorrectMaximumVelocityForAGearset();
  testMaxVelWorksOutOfOrder();
}
