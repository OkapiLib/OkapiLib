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

void ChassisController::left(const int ispeed) const {
  model->left(ispeed);
}

void ChassisController::right(const int ispeed) const {
  model->right(ispeed);
}

std::valarray<int> ChassisController::getSensorVals() const {
  return model->getSensorVals();
}

void ChassisController::resetSensors() const {
  model->resetSensors();
}

void ChassisController::setBrakeMode(const motor_brake_mode_e_t mode) const {
  model->setBrakeMode(mode);
}

void ChassisController::setEncoderUnits(const motor_encoder_units_e_t units) const {
  model->setEncoderUnits(units);
}

void ChassisController::setGearing(const motor_gearset_e_t gearset) const {
  model->setGearing(gearset);
}
} // namespace okapi
