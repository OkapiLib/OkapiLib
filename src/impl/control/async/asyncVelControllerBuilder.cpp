/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncVelControllerBuilder.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <stdexcept>

namespace okapi {
AsyncVelControllerBuilder::AsyncVelControllerBuilder() : logger(Logger::instance()) {
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

std::shared_ptr<AsyncVelocityController<double, double>> AsyncVelControllerBuilder::build() {
  if (!hasMotors) {
    logger->error("AsyncVelControllerBuilder: No motors given.");
    throw std::runtime_error("AsyncVelControllerBuilder: No motors given.");
  }

  if (hasGains) {
    if (!hasVelMath) {
      logger->error("AsyncVelControllerBuilder: No VelMath given.");
      throw std::runtime_error("AsyncVelControllerBuilder: No VelMath given.");
    }

    return buildAVPC();
  } else {
    return buildAVIC();
  }
}

std::shared_ptr<AsyncVelIntegratedController> AsyncVelControllerBuilder::buildAVIC() {
  return std::make_shared<AsyncVelIntegratedController>(motor, timeUtilFactory.create());
}

std::shared_ptr<AsyncVelPIDController> AsyncVelControllerBuilder::buildAVPC() {
  auto out = std::make_shared<AsyncVelPIDController>(sensor,
                                                     motor,
                                                     timeUtilFactory.create(),
                                                     gains.kP,
                                                     gains.kD,
                                                     gains.kF,
                                                     gains.kSF,
                                                     std::move(velMath),
                                                     std::move(derivativeFilter));
  out->startThread();
  return out;
}
} // namespace okapi
