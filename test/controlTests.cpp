/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/controlTests.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "test/crossPlatformTestRunner.hpp"

void testControlUtils() {
  using namespace okapi;
  using namespace snowhouse;


  {
    test_printf("Testing FlywheelSimulator");

    FlywheelSimulator sim;

    sim.setTorque(0.3);
    sim.step();

    test("FlywheelSimulator i = 0 angle",
         TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(0.000020193, 0.00000005)));
    test("FlywheelSimulator i = 0 omega",
         TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(0.0020193, 0.000005)));
    test("FlywheelSimulator i = 0 accel",
         TEST_BODY(AssertThat, sim.getAcceleration(), EqualsWithDelta(20.193, 0.0005)));
  }
}
