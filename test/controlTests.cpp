/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include <gtest/gtest.h>

using namespace okapi;

TEST(FlywheelSimulatorTest, BasicTest) {
  FlywheelSimulator sim;

  sim.setTorque(0.3);
  sim.step();

  EXPECT_NEAR(sim.getAngle(), 0.000020193, 0.00000005);
  EXPECT_NEAR(sim.getOmega(), 0.0020193, 0.000005);
  EXPECT_NEAR(sim.getAcceleration(), 20.193, 0.0005);
}
