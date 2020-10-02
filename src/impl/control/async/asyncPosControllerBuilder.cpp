/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncPosControllerBuilder.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <stdexcept>

namespace okapi {
AsyncPosControllerBuilder::AsyncPosControllerBuilder(const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger) {
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::withMotor(const Motor &imotor) {
  return withMotor(std::make_shared<Motor>(imotor));
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::withMotor(const MotorGroup &imotor) {
  return withMotor(std::make_shared<MotorGroup>(imotor));
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withMotor(const std::shared_ptr<AbstractMotor> &imotor) {
  hasMotors = true;

  motor = imotor;

  if (!sensorsSetByUser) {
    sensor = imotor->getEncoder();
  }

  if (!maxVelSetByUser) {
    maxVelocity = toUnderlyingType(imotor->getGearing());
  }

  if (!gearsetSetByUser) {
    pair = imotor->getGearing();
  }

  return *this;
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::withSensor(const ADIEncoder &isensor) {
  return withSensor(std::make_shared<ADIEncoder>(isensor));
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::withSensor(const IntegratedEncoder &isensor) {
  return withSensor(std::make_shared<IntegratedEncoder>(isensor));
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withSensor(const std::shared_ptr<RotarySensor> &isensor) {
  sensorsSetByUser = true;
  sensor = isensor;
  return *this;
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withGains(const IterativePosPIDController::Gains &igains) {
  hasGains = true;
  gains = igains;
  return *this;
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withDerivativeFilter(std::unique_ptr<Filter> iderivativeFilter) {
  derivativeFilter = std::move(iderivativeFilter);
  return *this;
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withGearset(const AbstractMotor::GearsetRatioPair &igearset) {
  gearsetSetByUser = true;
  pair = igearset;
  return *this;
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::withMaxVelocity(double imaxVelocity) {
  maxVelSetByUser = true;
  maxVelocity = imaxVelocity;
  return *this;
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
  timeUtilFactory = itimeUtilFactory;
  return *this;
}

AsyncPosControllerBuilder &
AsyncPosControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
  controllerLogger = ilogger;
  return *this;
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::parentedToCurrentTask() {
  isParentedToCurrentTask = true;
  return *this;
}

AsyncPosControllerBuilder &AsyncPosControllerBuilder::notParentedToCurrentTask() {
  isParentedToCurrentTask = false;
  return *this;
}

std::shared_ptr<AsyncPositionController<double, double>> AsyncPosControllerBuilder::build() {
  if (!hasMotors) {
    std::string msg("AsyncPosControllerBuilder: No motors given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!gearsetSetByUser) {
    LOG_WARN_S("AsyncPosControllerBuilder: The default gearset is selected. This could be a bug.");
  }

  if (hasGains) {
    return buildAPPC();
  } else {
    return buildAPIC();
  }
}

std::shared_ptr<AsyncPosIntegratedController> AsyncPosControllerBuilder::buildAPIC() {
  return std::make_shared<AsyncPosIntegratedController>(
    motor, pair, maxVelocity, timeUtilFactory.create(), controllerLogger);
}

std::shared_ptr<AsyncPosPIDController> AsyncPosControllerBuilder::buildAPPC() {
  motor->setGearing(pair.internalGearset);
  auto out = std::make_shared<AsyncPosPIDController>(sensor,
                                                     motor,
                                                     timeUtilFactory.create(),
                                                     gains.kP,
                                                     gains.kI,
                                                     gains.kD,
                                                     gains.kBias,
                                                     pair.ratio,
                                                     std::move(derivativeFilter),
                                                     controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}
} // namespace okapi
