/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "main.h"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

void testForwardUsesCorrectMaximumVelocityForAGearset() {
  printf("Testing forward uses the correct max vel for a gearset");

  auto drive = ChassisControllerBuilder()
                 .withMotors(18, 19)
                 .withGearset(AbstractMotor::gearset::green)
                 .withDimensions({1, 1})
                 .build();

  drive->forward(0.1);
  pros::delay(250);

  auto sampleRPM = Motor(18).getActualVelocity();
  test("Sample RPM should be approximately 20",
       TEST_BODY(AssertThat, sampleRPM, EqualsWithDelta(20, 2)));
}

void runChassisControllerFactoryIntegrationTests() {
  test_printf("Testing ChassisControllerBuilder");
}
