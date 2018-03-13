/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/chassisController.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisController::~ChassisController() = default;

void ChassisController::driveForward(const int ipower) {
  model.driveForward(ipower);
}

void ChassisController::driveVector(const int idistPower, const int ianglePower) {
  model.driveVector(idistPower, ianglePower);
}

void ChassisController::turnClockwise(const int ipower) {
  model.turnClockwise(ipower);
}

void ChassisController::stop() {
  model.stop();
}

void ChassisController::tank(const int ileftVal, const int irightVal, const int ithreshold) {
  model.tank(ileftVal, irightVal, ithreshold);
}

void ChassisController::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) {
  model.arcade(iverticalVal, ihorizontalVal, ithreshold);
}

void ChassisController::left(const int ipower) {
  model.left(ipower);
}

void ChassisController::right(const int ipower) {
  model.right(ipower);
}

std::valarray<int> ChassisController::getSensorVals() {
  return model.getSensorVals();
}

void ChassisController::resetSensors() const {
  model.resetSensors();
}
} // namespace okapi
