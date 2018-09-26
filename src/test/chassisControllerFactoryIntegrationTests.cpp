/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "main.h"

using namespace okapi;

void testForwardUsesCorrectMaximumVelocityForAGearset() {
  auto drive = ChassisControllerFactory::create(18, 19, AbstractMotor::gearset::green, {1, 1});
  drive.forward(0.1);
  Motor mtr(18);
  mtr.getActualVelocity(); // assert that this is ~20 RPM
}
