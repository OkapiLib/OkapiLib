/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/asyncPosIntegratedControllerTests.hpp"
#include "okapi/api.hpp"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

void testTarePosition() {
  printf("Testing tarePosition\n");

  const int moveAmt = 200;
  auto m = std::make_shared<Motor>(MOTOR_1_PORT);
  AsyncPosIntegratedController c(m, TimeUtilFactory::create());

  auto posBefore = m->getPosition();

  c.setTarget(moveAmt);
  c.waitUntilSettled();

  auto posAfter = m->getPosition();
  auto deltaPos = posAfter - posBefore;

  test("First move should produce a delta of " + std::to_string(moveAmt),
       TEST_BODY(AssertThat, deltaPos, EqualsWithDelta(moveAmt, 10)));

  c.tarePosition();

  posBefore = m->getPosition();

  c.setTarget(200);
  c.waitUntilSettled();

  posAfter = m->getPosition();
  deltaPos = posAfter - posBefore;

  test("Second move should produce a delta of " + std::to_string(moveAmt),
       TEST_BODY(AssertThat, deltaPos, EqualsWithDelta(moveAmt, 10)));
}

void runAsyncPosIntegratedControllerTests() {
  test_printf("Testing AsyncPosIntegratedController");
  testTarePosition();
}