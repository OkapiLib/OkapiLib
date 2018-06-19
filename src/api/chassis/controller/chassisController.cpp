/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/util/timer.hpp"
#include <cmath>

namespace okapi {
ChassisController::ChassisController(std::shared_ptr<ChassisModel> imodel) : model(imodel) {
}

ChassisController::~ChassisController() = default;

void ChassisController::forward(const int ispeed) const {
  model->forward(ispeed);
}

void ChassisController::driveVector(const double iySpeed, const double izRotation) const {
  model->driveVector(iySpeed, izRotation);
}

void ChassisController::rotate(const int ispeed) const {
  model->rotate(ispeed);
}

void ChassisController::stop() const {
  model->stop();
}

void ChassisController::tank(const double ileftSpeed, const double irightSpeed,
                             const double ithreshold) const {
  model->tank(ileftSpeed, irightSpeed, ithreshold);
}

void ChassisController::arcade(const double iySpeed, const double izRotation,
                               const double ithreshold) const {
  model->arcade(iySpeed, izRotation, ithreshold);
}

void ChassisController::left(const double ispeed) const {
  model->left(ispeed);
}

void ChassisController::right(const double ispeed) const {
  model->right(ispeed);
}

std::valarray<std::int32_t> ChassisController::getSensorVals() const {
  return model->getSensorVals();
}

void ChassisController::resetSensors() const {
  model->resetSensors();
}

void ChassisController::setBrakeMode(const AbstractMotor::motorBrakeMode mode) const {
  model->setBrakeMode(mode);
}

void ChassisController::setEncoderUnits(const AbstractMotor::motorEncoderUnits units) const {
  model->setEncoderUnits(units);
}

void ChassisController::setGearing(const AbstractMotor::motorGearset gearset) const {
  model->setGearing(gearset);
}
} // namespace okapi
