/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/chassis/controller/chassisController.hpp"
#include "okapi/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisController::ChassisController(const ChassisModel &imodel) : model(imodel) {
}

ChassisController::~ChassisController() = default;

void ChassisController::forward(const int ipower) const {
  model.forward(ipower);
}

void ChassisController::driveVector(const int idistPower, const int ianglePower) const {
  model.driveVector(idistPower, ianglePower);
}

void ChassisController::rotate(const int ipower) const {
  model.rotate(ipower);
}

void ChassisController::stop() const {
  model.stop();
}

void ChassisController::tank(const int ileftVal, const int irightVal, const int ithreshold) const {
  model.tank(ileftVal, irightVal, ithreshold);
}

void ChassisController::arcade(int iverticalVal, int ihorizontalVal, const int ithreshold) const {
  model.arcade(iverticalVal, ihorizontalVal, ithreshold);
}

void ChassisController::left(const int ipower) const {
  model.left(ipower);
}

void ChassisController::right(const int ipower) const {
  model.right(ipower);
}

std::valarray<int> ChassisController::getSensorVals() const {
  return model.getSensorVals();
}

void ChassisController::resetSensors() const {
  model.resetSensors();
}
} // namespace okapi
