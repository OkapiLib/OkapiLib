/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/chassisController.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisController::~ChassisController() = default;

void ChassisController::driveForward(const int ipower) const {
  model->driveForward(ipower);
}

void ChassisController::driveVector(const int idistPower, const int ianglePower) const {
  model->driveVector(idistPower, ianglePower);
}

void ChassisController::turnClockwise(const int ipower) const {
  model->turnClockwise(ipower);
}

void ChassisController::stop() const {
  model->stop();
}

void ChassisController::tank(const int ileftVal, const int irightVal, const int ithreshold) const {
  model->tank(ileftVal, irightVal, ithreshold);
}

void ChassisController::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) const {
  model->arcade(iverticalVal, ihorizontalVal, ithreshold);
}

void ChassisController::left(const int ipower) const {
  model->left(ipower);
}

void ChassisController::right(const int ipower) const {
  model->right(ipower);
}

std::valarray<int> ChassisController::getSensorVals() const {
  return model->getSensorVals();
}

void ChassisController::resetSensors() const {
  model->resetSensors();
}
} // namespace okapi
