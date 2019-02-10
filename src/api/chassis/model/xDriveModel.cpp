/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include <utility>

namespace okapi {
XDriveModel::XDriveModel(const std::shared_ptr<AbstractMotor> &itopLeftMotor,
                         const std::shared_ptr<AbstractMotor> &itopRightMotor,
                         const std::shared_ptr<AbstractMotor> &ibottomRightMotor,
                         const std::shared_ptr<AbstractMotor> &ibottomLeftMotor,
                         const std::shared_ptr<ContinuousRotarySensor> &ileftEnc,
                         const std::shared_ptr<ContinuousRotarySensor> &irightEnc,
                         const double imaxVelocity,
                         const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(ileftEnc),
    rightSensor(irightEnc) {
}

XDriveModel::XDriveModel(const std::shared_ptr<AbstractMotor> &itopLeftMotor,
                         const std::shared_ptr<AbstractMotor> &itopRightMotor,
                         const std::shared_ptr<AbstractMotor> &ibottomRightMotor,
                         const std::shared_ptr<AbstractMotor> &ibottomLeftMotor,
                         const double imaxVelocity,
                         const double imaxVoltage)
  : ChassisModel::ChassisModel(imaxVelocity, imaxVoltage),
    topLeftMotor(itopLeftMotor),
    topRightMotor(itopRightMotor),
    bottomRightMotor(ibottomRightMotor),
    bottomLeftMotor(ibottomLeftMotor),
    leftSensor(itopLeftMotor->getEncoder()),
    rightSensor(itopRightMotor->getEncoder()) {
}

void XDriveModel::forward(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::driveVector(const double iforwardSpeed, const double iyaw) const {
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

  topLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
}

void XDriveModel::rotate(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  topRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(-1 * speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::stop() {
  topLeftMotor->moveVelocity(0);
  topRightMotor->moveVelocity(0);
  bottomRightMotor->moveVelocity(0);
  bottomLeftMotor->moveVelocity(0);
}

void XDriveModel::tank(const double ileftSpeed,
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

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightSpeed * maxVoltage));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftSpeed * maxVoltage));
}

void XDriveModel::arcade(const double iforwardSpeed,
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

  leftOutput = std::clamp(leftOutput, -1.0, 1.0);
  rightOutput = std::clamp(rightOutput, -1.0, 1.0);

  topLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
  topRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomRightMotor->moveVoltage(static_cast<int16_t>(rightOutput * maxVoltage));
  bottomLeftMotor->moveVoltage(static_cast<int16_t>(leftOutput * maxVoltage));
}

void XDriveModel::xArcade(const double ixSpeed,
                          const double iforwardSpeed,
                          const double iyaw,
                          const double ithreshold) const {
  double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
  if (std::abs(xSpeed) < ithreshold) {
    xSpeed = 0;
  }

  double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
  if (std::abs(forwardSpeed) < ithreshold) {
    forwardSpeed = 0;
  }

  double yaw = std::clamp(iyaw, -1.0, 1.0);
  if (std::abs(yaw) < ithreshold) {
    yaw = 0;
  }

  topLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed + yaw, -1.0, 1.0) * maxVoltage));
  topRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed - yaw, -1.0, 1.0) * maxVoltage));
  bottomRightMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed - yaw, -1.0, 1.0) * maxVoltage));
  bottomLeftMotor->moveVoltage(
    static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed + yaw, -1.0, 1.0) * maxVoltage));
}

void XDriveModel::left(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomLeftMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

void XDriveModel::right(const double ispeed) const {
  const double speed = std::clamp(ispeed, -1.0, 1.0);
  topRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
  bottomRightMotor->moveVelocity(static_cast<int16_t>(speed * maxVelocity));
}

std::valarray<std::int32_t> XDriveModel::getSensorVals() const {
  return std::valarray<std::int32_t>{static_cast<std::int32_t>(leftSensor->get()),
                                     static_cast<std::int32_t>(rightSensor->get())};
}

void XDriveModel::resetSensors() const {
  leftSensor->reset();
  rightSensor->reset();
}

void XDriveModel::setBrakeMode(const AbstractMotor::brakeMode mode) const {
  topLeftMotor->setBrakeMode(mode);
  topRightMotor->setBrakeMode(mode);
  bottomRightMotor->setBrakeMode(mode);
  bottomLeftMotor->setBrakeMode(mode);
}

void XDriveModel::setEncoderUnits(const AbstractMotor::encoderUnits units) const {
  topLeftMotor->setEncoderUnits(units);
  topRightMotor->setEncoderUnits(units);
  bottomRightMotor->setEncoderUnits(units);
  bottomLeftMotor->setEncoderUnits(units);
}

void XDriveModel::setGearing(const AbstractMotor::gearset gearset) const {
  topLeftMotor->setGearing(gearset);
  topRightMotor->setGearing(gearset);
  bottomRightMotor->setGearing(gearset);
  bottomLeftMotor->setGearing(gearset);
}

void XDriveModel::setPosPID(const double ikF,
                            const double ikP,
                            const double ikI,
                            const double ikD) const {
  topLeftMotor->setPosPID(ikF, ikP, ikI, ikD);
  topRightMotor->setPosPID(ikF, ikP, ikI, ikD);
  bottomRightMotor->setPosPID(ikF, ikP, ikI, ikD);
  bottomLeftMotor->setPosPID(ikF, ikP, ikI, ikD);
}

void XDriveModel::setPosPIDFull(const double ikF,
                                const double ikP,
                                const double ikI,
                                const double ikD,
                                const double ifilter,
                                const double ilimit,
                                const double ithreshold,
                                const double iloopSpeed) const {
  topLeftMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  topRightMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomRightMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomLeftMotor->setPosPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

void XDriveModel::setVelPID(const double ikF,
                            const double ikP,
                            const double ikI,
                            const double ikD) const {
  topLeftMotor->setVelPID(ikF, ikP, ikI, ikD);
  topRightMotor->setVelPID(ikF, ikP, ikI, ikD);
  bottomRightMotor->setVelPID(ikF, ikP, ikI, ikD);
  bottomLeftMotor->setVelPID(ikF, ikP, ikI, ikD);
}

void XDriveModel::setVelPIDFull(const double ikF,
                                const double ikP,
                                const double ikI,
                                const double ikD,
                                const double ifilter,
                                const double ilimit,
                                const double ithreshold,
                                const double iloopSpeed) const {
  topLeftMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  topRightMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomRightMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
  bottomLeftMotor->setVelPIDFull(ikF, ikP, ikI, ikD, ifilter, ilimit, ithreshold, iloopSpeed);
}

std::shared_ptr<AbstractMotor> XDriveModel::getTopLeftMotor() const {
  return topLeftMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getTopRightMotor() const {
  return topRightMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getBottomRightMotor() const {
  return bottomRightMotor;
}

std::shared_ptr<AbstractMotor> XDriveModel::getBottomLeftMotor() const {
  return bottomLeftMotor;
}
} // namespace okapi
