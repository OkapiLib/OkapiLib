/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncVelControllerBuilder.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <stdexcept>

namespace okapi {
AsyncVelControllerBuilder::AsyncVelControllerBuilder(const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger) {
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::withMotor(const Motor &imotor) {
  return withMotor(std::make_shared<Motor>(imotor));
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::withMotor(const MotorGroup &imotor) {
  return withMotor(std::make_shared<MotorGroup>(imotor));
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withMotor(const std::shared_ptr<AbstractMotor> &imotor) {
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

AsyncVelControllerBuilder &AsyncVelControllerBuilder::withSensor(const ADIEncoder &isensor) {
  return withSensor(std::make_shared<ADIEncoder>(isensor));
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::withSensor(const IntegratedEncoder &isensor) {
  return withSensor(std::make_shared<IntegratedEncoder>(isensor));
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withSensor(const std::shared_ptr<RotarySensor> &isensor) {
  sensorsSetByUser = true;
  sensor = isensor;
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withGains(const IterativeVelPIDController::Gains &igains) {
  hasGains = true;
  gains = igains;
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withVelMath(std::unique_ptr<VelMath> ivelMath) {
  hasVelMath = true;
  velMath = std::move(ivelMath);
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withDerivativeFilter(std::unique_ptr<Filter> iderivativeFilter) {
  derivativeFilter = std::move(iderivativeFilter);
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withGearset(const AbstractMotor::GearsetRatioPair &igearset) {
  gearsetSetByUser = true;
  pair = igearset;
  return *this;
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::withMaxVelocity(double imaxVelocity) {
  maxVelSetByUser = true;
  maxVelocity = imaxVelocity;
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
  timeUtilFactory = itimeUtilFactory;
  return *this;
}

AsyncVelControllerBuilder &
AsyncVelControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
  controllerLogger = ilogger;
  return *this;
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::parentedToCurrentTask() {
  isParentedToCurrentTask = true;
  return *this;
}

AsyncVelControllerBuilder &AsyncVelControllerBuilder::notParentedToCurrentTask() {
  isParentedToCurrentTask = false;
  return *this;
}

std::shared_ptr<AsyncVelocityController<double, double>> AsyncVelControllerBuilder::build() {
  if (!hasMotors) {
    std::string msg("AsyncVelControllerBuilder: No motors given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (hasGains) {
    if (!hasVelMath) {
      std::string msg("AsyncVelControllerBuilder: No VelMath given.");
      LOG_ERROR(msg);
      throw std::runtime_error(msg);
    }

    if (!gearsetSetByUser) {
      LOG_WARN_S(
        "AsyncVelControllerBuilder: The default gearset is selected. This could be a bug.");
    }

    return buildAVPC();
  } else {
    return buildAVIC();
  }
}

std::shared_ptr<AsyncVelIntegratedController> AsyncVelControllerBuilder::buildAVIC() {
  return std::make_shared<AsyncVelIntegratedController>(
    motor, pair, maxVelocity, timeUtilFactory.create(), controllerLogger);
}

std::shared_ptr<AsyncVelPIDController> AsyncVelControllerBuilder::buildAVPC() {
  motor->setGearing(pair.internalGearset);
  auto out = std::make_shared<AsyncVelPIDController>(sensor,
                                                     motor,
                                                     timeUtilFactory.create(),
                                                     gains.kP,
                                                     gains.kD,
                                                     gains.kF,
                                                     gains.kSF,
                                                     std::move(velMath),
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
