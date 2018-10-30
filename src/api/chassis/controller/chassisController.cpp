/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisController.hpp"
#include <cmath>

namespace okapi {
ChassisController::ChassisController(const std::shared_ptr<ChassisModel> &imodel,
                                     const double imaxVelocity,
                                     const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage), model(imodel) {
}

ChassisController::~ChassisController() = default;

void ChassisController::setTurnsMirrored(const bool ishouldMirror) {
  normalTurns = !ishouldMirror;
}

void ChassisController::forward(const double ispeed) const {
  model->forward(ispeed);
}

void ChassisController::driveVector(const double iforwardSpeed, const double iyaw) const {
  model->driveVector(iforwardSpeed, iyaw);
}

void ChassisController::rotate(const double ispeed) const {
  model->rotate(ispeed);
}

void ChassisController::stop() {
  model->stop();
}

void ChassisController::tank(const double ileftSpeed,
                             const double irightSpeed,
                             const double ithreshold) const {
  model->tank(ileftSpeed, irightSpeed, ithreshold);
}

void ChassisController::arcade(const double iforwardSpeed,
                               const double iyaw,
                               const double ithreshold) const {
  model->arcade(iforwardSpeed, iyaw, ithreshold);
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

void ChassisController::setBrakeMode(const AbstractMotor::brakeMode mode) const {
  model->setBrakeMode(mode);
}

void ChassisController::setEncoderUnits(const AbstractMotor::encoderUnits units) const {
  model->setEncoderUnits(units);
}

void ChassisController::setGearing(const AbstractMotor::gearset gearset) const {
  model->setGearing(gearset);
}

void ChassisController::setPosPID(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD) const {
  model->setPosPID(ikF, ikP, ikI, ikD);
}

void ChassisController::setPosPIDFull(const double ikF,
                                      const double ikP,
                                      const double ikI,
                                      const double ikD,
                                      const double ifilter,
                                      const double ilimit,
                                      const double ithreshold,
                                      const double iloopSpeed) const {
  model->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void ChassisController::setVelPID(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD) const {
  model->setVelPID(ikF, ikP, ikI, ikD);
}

void ChassisController::setVelPIDFull(const double ikF,
                                      const double ikP,
                                      const double ikI,
                                      const double ikD,
                                      const double ifilter,
                                      const double ilimit,
                                      const double ithreshold,
                                      const double iloopSpeed) const {
  model->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void ChassisController::setMaxVelocity(double imaxVelocity) {
  model->setMaxVelocity(imaxVelocity);
}

void ChassisController::setMaxVoltage(double imaxVoltage) {
  model->setMaxVoltage(imaxVoltage);
}

std::shared_ptr<ChassisModel> ChassisController::getChassisModel() const {
  return model;
}
} // namespace okapi
