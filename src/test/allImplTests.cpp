/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/allImplTests.hpp"
#include "test/testRunner.hpp"
#include "test/tests/impl/asyncMotionProfileControllerBuilderIntegrationTests.hpp"
#include "test/tests/impl/asyncPosControllerBuilderIntegrationTests.hpp"
#include "test/tests/impl/asyncPosIntegratedControllerTests.hpp"
#include "test/tests/impl/asyncVelControllerBuilderIntegrationTests.hpp"
#include "test/tests/impl/chassisControllerBuilderIntegrationTests.hpp"
#include "test/tests/impl/chassisControllerIntegratedTests.hpp"
#include "test/tests/impl/chassisControllerPidTests.hpp"
#include "test/tests/impl/controllerTests.hpp"
#include "test/tests/impl/utilTests.hpp"

using namespace okapi;

void runAllImplTests() {
  //  runChassisControllerBuilderIntegrationTests();
  //  runAsyncPosControllerBuilderIntegrationTests();
  //  runAsyncMotionProfileControllerBuilderIntegrationTests();
  //  runAsyncPosIntegratedControllerTests();
  //  runAsyncVelControllerBuilderIntegrationTests();
  //  runUtilTests();
  runControllerTests();
  //  runChassisControllerPidTests();
  //  runChassisControllerIntegratedTests();
  test_print_report();
}
