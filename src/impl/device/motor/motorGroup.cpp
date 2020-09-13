/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/device/motor/motorGroup.hpp"

namespace okapi {
MotorGroup::MotorGroup(const std::initializer_list<Motor> &imotors,
                       const std::shared_ptr<Logger> &logger) {
  if (imotors.size() == 0) {
    std::string msg(
      "MotorGroup: A MotorGroup must be created with at least one motor. No motors were given.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }

  motors.reserve(imotors.size());
  for (auto motor : imotors) {
    motors.push_back(std::make_shared<Motor>(motor));
  }
}

MotorGroup::MotorGroup(const std::initializer_list<std::shared_ptr<AbstractMotor>> &imotors,
                       const std::shared_ptr<Logger> &logger)
  : motors(imotors) {
  if (motors.empty()) {
    std::string msg(
      "MotorGroup: A MotorGroup must be created with at least one motor. No motors were given.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}

std::int32_t MotorGroup::moveAbsolute(const double iposition, const std::int32_t ivelocity) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->moveAbsolute(iposition, ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveRelative(const double iposition, const std::int32_t ivelocity) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->moveRelative(iposition, ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveVelocity(const std::int16_t ivelocity) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->moveVelocity(ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::moveVoltage(const std::int16_t ivoltage) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->moveVoltage(ivoltage);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::modifyProfiledVelocity(std::int32_t ivelocity) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->modifyProfiledVelocity(ivelocity);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

double MotorGroup::getTargetPosition() {
  return motors[0]->getTargetPosition();
}

double MotorGroup::getPosition() {
  return motors[0]->getPosition();
}

std::int32_t MotorGroup::tarePosition() {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->tarePosition();
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::getTargetVelocity() {
  return motors[0]->getTargetVelocity();
}

double MotorGroup::getActualVelocity() {
  return motors[0]->getActualVelocity();
}

std::int32_t MotorGroup::getCurrentDraw() {
  return motors[0]->getCurrentDraw();
}

std::int32_t MotorGroup::getDirection() {
  return motors[0]->getDirection();
}

double MotorGroup::getEfficiency() {
  return motors[0]->getEfficiency();
}

std::int32_t MotorGroup::isOverCurrent() {
  return motors[0]->isOverCurrent();
}

std::int32_t MotorGroup::isOverTemp() {
  return motors[0]->isOverTemp();
}

std::int32_t MotorGroup::isStopped() {
  return motors[0]->isStopped();
}

std::int32_t MotorGroup::getZeroPositionFlag() {
  return motors[0]->getZeroPositionFlag();
}

uint32_t MotorGroup::getFaults() {
  return motors[0]->getFaults();
}

uint32_t MotorGroup::getFlags() {
  return motors[0]->getFlags();
}

std::int32_t MotorGroup::getRawPosition(std::uint32_t *timestamp) {
  return motors[0]->getRawPosition(timestamp);
}

double MotorGroup::getPower() {
  return motors[0]->getPower();
}

double MotorGroup::getTemperature() {
  return motors[0]->getTemperature();
}

double MotorGroup::getTorque() {
  return motors[0]->getTorque();
}

std::int32_t MotorGroup::getVoltage() {
  return motors[0]->getVoltage();
}

std::int32_t MotorGroup::setBrakeMode(const AbstractMotor::brakeMode imode) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setBrakeMode(imode);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::brakeMode MotorGroup::getBrakeMode() {
  return motors[0]->getBrakeMode();
}

std::int32_t MotorGroup::setCurrentLimit(const std::int32_t ilimit) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setCurrentLimit(ilimit);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::getCurrentLimit() {
  return motors[0]->getCurrentLimit();
}

std::int32_t MotorGroup::setEncoderUnits(const AbstractMotor::encoderUnits iunits) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setEncoderUnits(iunits);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::encoderUnits MotorGroup::getEncoderUnits() {
  return motors[0]->getEncoderUnits();
}

std::int32_t MotorGroup::setGearing(const AbstractMotor::gearset igearset) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setGearing(igearset);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

AbstractMotor::gearset MotorGroup::getGearing() {
  return motors[0]->getGearing();
}

std::int32_t MotorGroup::setReversed(const bool ireverse) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setReversed(ireverse);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

std::int32_t MotorGroup::setVoltageLimit(const std::int32_t ilimit) {
  auto out = 1;
  for (auto &&elem : motors) {
    const auto errorCode = elem->setVoltageLimit(ilimit);
    if (errorCode != 1) {
      out = errorCode;
    }
  }
  return out;
}

void MotorGroup::controllerSet(const double ivalue) {
  for (auto &&elem : motors) {
    elem->controllerSet(ivalue);
  }
}

std::shared_ptr<ContinuousRotarySensor> MotorGroup::getEncoder() {
  return getEncoder(0);
}

std::shared_ptr<ContinuousRotarySensor> MotorGroup::getEncoder(const std::size_t index) {
  return motors[index]->getEncoder();
}
} // namespace okapi
