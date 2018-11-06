/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/allImplTests.hpp"
#include "test/tests/impl/asyncPosIntegratedControllerTests.hpp"
#include "test/tests/impl/controllerTests.hpp"
#include "test/tests/impl/utilTests.hpp"

void runAllImplTests() {
  runAsyncPosIntegratedControllerTests();
  runUtilTests();
  runControllerTests();
}
