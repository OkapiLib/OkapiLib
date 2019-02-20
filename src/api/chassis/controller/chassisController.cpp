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

void ChassisController::forward(const double ispeed) {
  model->forward(ispeed);
}

void ChassisController::driveVector(const double iforwardSpeed, const double iyaw) {
  model->driveVector(iforwardSpeed, iyaw);
}

void ChassisController::driveVectorVoltage(const double iforwardSpeed, const double iyaw) {
  model->driveVectorVoltage(iforwardSpeed, iyaw);
}

void ChassisController::rotate(const double ispeed) {
  model->rotate(ispeed);
}

void ChassisController::stop() {
  model->stop();
}

void ChassisController::tank(const double ileftSpeed,
                             const double irightSpeed,
                             const double ithreshold) {
  model->tank(ileftSpeed, irightSpeed, ithreshold);
}

void ChassisController::arcade(const double iforwardSpeed,
                               const double iyaw,
                               const double ithreshold) {
  model->arcade(iforwardSpeed, iyaw, ithreshold);
}

void ChassisController::left(const double ispeed) {
  model->left(ispeed);
}

void ChassisController::right(const double ispeed) {
  model->right(ispeed);
}

std::valarray<std::int32_t> ChassisController::getSensorVals() const {
  return model->getSensorVals();
}

void ChassisController::resetSensors() {
  model->resetSensors();
}

void ChassisController::setBrakeMode(const AbstractMotor::brakeMode mode) {
  model->setBrakeMode(mode);
}

void ChassisController::setEncoderUnits(const AbstractMotor::encoderUnits units) {
  model->setEncoderUnits(units);
}

void ChassisController::setGearing(const AbstractMotor::gearset gearset) {
  model->setGearing(gearset);
}

void ChassisController::setPosPID(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD) {
  model->setPosPID(ikF, ikP, ikI, ikD);
}

void ChassisController::setPosPIDFull(const double ikF,
                                      const double ikP,
                                      const double ikI,
                                      const double ikD,
                                      const double ifilter,
                                      const double ilimit,
                                      const double ithreshold,
                                      const double iloopSpeed) {
  model->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void ChassisController::setVelPID(const double ikF,
                                  const double ikP,
                                  const double ikI,
                                  const double ikD) {
  model->setVelPID(ikF, ikP, ikI, ikD);
}

void ChassisController::setVelPIDFull(const double ikF,
                                      const double ikP,
                                      const double ikI,
                                      const double ikD,
                                      const double ifilter,
                                      const double ilimit,
                                      const double ithreshold,
                                      const double iloopSpeed) {
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
