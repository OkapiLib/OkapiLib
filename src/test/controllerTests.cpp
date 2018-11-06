/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/controllerTests.hpp"
#include "okapi/api.hpp"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

void testBtnOperatorOverloadReturnsTheSameInstance() {
  printf("Testing Controller operator[] overload returns the same ControllerButton instance\n");

  Controller c;
  auto btnAddr1 = &c[ControllerDigital::X];
  auto btnAddr2 = &c[ControllerDigital::X];

  test("First button is not null", TEST_BODY(AssertThat, btnAddr1, Is().Not().Null()));
  test("Two calls should give the same address",
       TEST_BODY(AssertThat, btnAddr1 == btnAddr2, IsTrue()));
}

void runControllerTests() {
  test_printf("Testing Controller");
  testBtnOperatorOverloadReturnsTheSameInstance();
}
