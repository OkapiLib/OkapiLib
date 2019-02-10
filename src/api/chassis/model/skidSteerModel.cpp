/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include <algorithm>
#include <utility>

namespace okapi {
SkidSteerModel::SkidSteerModel(const std::shared_ptr<AbstractMotor> &ileftSideMotor,
                               const std::shared_ptr<AbstractMotor> &irightSideMotor,
                               const std::shared_ptr<ContinuousRotarySensor> &ileftEnc,
                               const std::shared_ptr<ContinuousRotarySensor> &irightEnc,
                               const double imaxVelocity,
                               const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

SkidSteerModel::SkidSteerModel(const std::shared_ptr<AbstractMotor> &ileftSideMotor,
                               const std::shared_ptr<AbstractMotor> &irightSideMotor,
                               const double imaxVelocity,
                               const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    leftSideMotor(ileftSideMotor),
    rightSideMotor(irightSideMotor),
    leftSensor(ileftSideMotor->getEncoder()),
    rightSensor(irightSideMotor->getEncoder()) {
}

void SkidSteerModel::forward(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  leftSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void SkidSteerModel::driveVector(const double iforwardSpeed, const double iyaw) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  const double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  const double yaw = std::clamp(iyaw, -1.0, 1.0);

  double leftOutput = forwardSpeed + yaw;
  double rightOutput = forwardSpeed - yaw;
  if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
      maxInputMag > 1) {
    leftOutput /= maxInputMag;
    rightOutput /= maxInputMag;
  }

  leftSideMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
}

void SkidSteerModel::rotate(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  leftSideMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  rightSideMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
}

void SkidSteerModel::stop() {
  leftSideMotor->moveVelocity(0);
  rightSideMotor->moveVelocity(0);
}

void SkidSteerModel::tank(const double ileftSpeed,
                          const double irightSpeed,
                          const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double leftSpeed = std::clamp(ileftSpeed, -1.0, 1.0);
  if (std::abs(leftSpeed) < ithreshold) {
    leftSpeed = 0;
  }

  double rightSpeed = std::clamp(irightSpeed, -1.0, 1.0);
  if (std::abs(rightSpeed) < ithreshold) {
    rightSpeed = 0;
  }

  leftSideMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
  rightSideMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
}

void SkidSteerModel::arcade(const double iforwardSpeed,
                            const double iyaw,
                            const double ithreshold) const {
  // This code is taken from WPIlib. All credit goes to them. Link:
  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
  double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  if (std::abs(forwardSpeed) <= ithreshold) {
    forwardSpeed = 0;
  }

  double yaw = std::clamp(iyaw, -1.0, 1.0);
  if (std::abs(yaw) <= ithreshold) {
    yaw = 0;
  }

  double maxInput = std::copysign(std::max(std::abs(forwardSpeed), std::abs(yaw)), forwardSpeed);
  double leftOutput = 0;
  double rightOutput = 0;

  if (forwardSpeed >= 0) {
    if (yaw >= 0) {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    } else {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    }
  } else {
    if (yaw >= 0) {
      leftOutput = forwardSpeed + yaw;
      rightOutput = maxInput;
    } else {
      leftOutput = maxInput;
      rightOutput = forwardSpeed - yaw;
    }
  }

  leftSideMotor->moveVoltage(static_cast<int16_t>(std::clamp(leftOutput, -1.0, 1.0) * maxVoltage));
  rightSideMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(rightOutput, -1.0, 1.0) * maxVoltage));
}

void SkidSteerModel::left(const double ispeed) const {
  leftSideMotor->moveVelocity(static_cast<int16_t>(std::clamp(ispeed, -1.0, 1.0) * maxVelocity));
}

void SkidSteerModel::right(const double ispeed) const {
  rightSideMotor->moveVelocity(static_cast<int16_t>(std::clamp(ispeed, -1.0, 1.0) * maxVelocity));
}

std::valarray<std::int32_t> SkidSteerModel::getSensorVals() const {
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get())};
}

void SkidSteerModel::resetSensors() const {
  leftSensor->reset();
  rightSensor->reset();
}

void SkidSteerModel::setBrakeMode(const AbstractMotor::brakeMode mode) const {
  leftSideMotor->setBrakeMode(mode);
  rightSideMotor->setBrakeMode(mode);
}

void SkidSteerModel::setEncoderUnits(const AbstractMotor::encoderUnits units) const {
  leftSideMotor->setEncoderUnits(units);
  rightSideMotor->setEncoderUnits(units);
}

void SkidSteerModel::setGearing(const AbstractMotor::gearset gearset) const {
  leftSideMotor->setGearing(gearset);
  rightSideMotor->setGearing(gearset);
}

void SkidSteerModel::setPosPID(const double ikF,
                               const double ikP,
                               const double ikI,
                               const double ikD) const {
  leftSideMotor->setPosPID(ikF, ikP, ikI, ikD);
  rightSideMotor->setPosPID(ikF, ikP, ikI, ikD);
}

void SkidSteerModel::setPosPIDFull(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ifilter,
                                   const double ilimit,
                                   const double ithreshold,
                                   const double iloopSpeed) const {
  leftSideMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  rightSideMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void SkidSteerModel::setVelPID(const double ikF,
                               const double ikP,
                               const double ikI,
                               const double ikD) const {
  leftSideMotor->setVelPID(ikF, ikP, ikI, ikD);
  rightSideMotor->setVelPID(ikF, ikP, ikI, ikD);
}

void SkidSteerModel::setVelPIDFull(const double ikF,
                                   const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ifilter,
                                   const double ilimit,
                                   const double ithreshold,
                                   const double iloopSpeed) const {
  leftSideMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  rightSideMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

std::shared_ptr<AbstractMotor> SkidSteerModel::getLeftSideMotor() const {
  return leftSideMotor;
}

std::shared_ptr<AbstractMotor> SkidSteerModel::getRightSideMotor() const {
  return rightSideMotor;
}
} // namespace okapi
