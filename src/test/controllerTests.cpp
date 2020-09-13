/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/controllerTests.hpp"
#include "test/testRunner.hpp"

using namespace snowhouse;
using namespace okapi;

static void testBtnOperatorOverloadReturnsTheSameInstanceOnMaster() {
  printf("Testing Controller operator[] overload returns the same ControllerButton instance on the "
         "master controller\n");
  resetHardware();

  Controller c;
  auto btnAddr1 = &c[ControllerDigital::X];
  auto btnAddr2 = &c[ControllerDigital::X];

  test("First button is not null", TEST_BODY(AssertThat, btnAddr1, Is().Not().Null()));
  test("Two calls should give the same address",
       TEST_BODY(AssertThat, btnAddr1 == btnAddr2, IsTrue()));
  resetHardware();
}

static void testBtnOperatorOverloadReturnsTheSameInstanceOnPartner() {
  printf("Testing Controller operator[] overload returns the same ControllerButton instance on the "
         "partner controller\n");
  resetHardware();

  Controller c(ControllerId::partner);
  auto btnAddr1 = &c[ControllerDigital::X];
  auto btnAddr2 = &c[ControllerDigital::X];

  test("First button is not null", TEST_BODY(AssertThat, btnAddr1, Is().Not().Null()));
  test("Two calls should give the same address",
       TEST_BODY(AssertThat, btnAddr1 == btnAddr2, IsTrue()));
  resetHardware();
}

static void testBtnOperatorOverloadWorksWithMasterAndPartnerSimultaneously() {
  printf("Testing Controller operator[] overload works for master and partner simultaneously\n");
  resetHardware();

  Controller master;
  Controller partner(ControllerId::partner);
  auto btnAddrMaster = &master[ControllerDigital::X];
  auto btnAddrPartner = &partner[ControllerDigital::X];

  test("The master button is not null", TEST_BODY(AssertThat, btnAddrMaster, Is().Not().Null()));
  test("The partner button is not null", TEST_BODY(AssertThat, btnAddrPartner, Is().Not().Null()));
  test("The two buttons have different addresses",
       TEST_BODY(AssertThat, btnAddrMaster != btnAddrPartner, IsTrue()));

  resetHardware();
}

void runControllerTests() {
  test_printf("Testing Controller");
  testBtnOperatorOverloadReturnsTheSameInstanceOnMaster();
  testBtnOperatorOverloadReturnsTheSameInstanceOnPartner();
  testBtnOperatorOverloadWorksWithMasterAndPartnerSimultaneously();
}
