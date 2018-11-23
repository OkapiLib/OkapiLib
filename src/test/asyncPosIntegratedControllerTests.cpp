/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncPosIntegratedControllerTests.hpp"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

static void testTarePosition() {
  printf("Testing tarePosition\n");
  resetHardware();

  const int moveAmt = 500;
  auto m = std::make_shared<Motor>(MOTOR_1_PORT);
  m->tarePosition();

  AsyncPosIntegratedController c(
    m, toUnderlyingType(MOTOR_GEARSET) / 2.0, TimeUtilFactory::withSettledUtilParams(10, 0, 0_ms));

  // Do one more normally
  auto posBefore = m->getPosition();

  c.setTarget(moveAmt);
  c.waitUntilSettled();

  auto posAfter = m->getPosition();
  auto deltaPos = posAfter - posBefore;

  test("First move should produce a delta of " + std::to_string(moveAmt),
       TEST_BODY(AssertThat, deltaPos, EqualsWithDelta(moveAmt, 10)));

  // Tare and do the same move again
  c.tarePosition();

  posBefore = m->getPosition();

  c.setTarget(moveAmt);
  c.waitUntilSettled();

  posAfter = m->getPosition();
  deltaPos = posAfter - posBefore;

  // Moving the same amount after a tare should produce the same delta
  test("Second move should produce a delta of " + std::to_string(moveAmt),
       TEST_BODY(AssertThat, deltaPos, EqualsWithDelta(moveAmt, 10)));

  c.flipDisable(true);
  resetHardware();
  pros::delay(500);
}

static void testStop() {
  printf("Testing stop\n");
  resetHardware();

  const int moveAmt = 500;
  const double requiredDeltaPos = moveAmt / 3.0;
  auto m = std::make_shared<Motor>(MOTOR_1_PORT);
  m->tarePosition();

  // Slow max speed so we don't coast too much after stopping
  AsyncPosIntegratedController c(m, 30, TimeUtilFactory::create());

  auto posBefore = m->getPosition();
  c.setTarget(moveAmt);

  // Wait for some movement to happen so we don't interrupt too quickly
  auto currentPos = m->getPosition();
  while (currentPos - posBefore <= requiredDeltaPos) {
    currentPos = m->getPosition();
    pros::delay(10);
  }

  c.stop();

  // Let the motor come to a full stop
  pros::delay(500);

  test("The motor position after stopping should be roughly equal to the position when stop was "
       "called",
       TEST_BODY(AssertThat, std::fabs(m->getPosition() - currentPos), EqualsWithDelta(0, 50)));

  // Try moving to the target again
  c.setTarget(moveAmt);
  c.waitUntilSettled();

  test("Moving to the target after being interrupted should work",
       TEST_BODY(AssertThat, m->getPosition(), EqualsWithDelta(moveAmt, 50)));

  c.flipDisable(true);
  resetHardware();
  pros::delay(500);
}

void runAsyncPosIntegratedControllerTests() {
  test_printf("Testing AsyncPosIntegratedController");
  testTarePosition();
  testStop();
}
