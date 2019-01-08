/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncMotionProfileControllerBuilder.hpp"

namespace okapi {
AsyncMotionProfileControllerBuilder::AsyncMotionProfileControllerBuilder(
  const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger) {
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const Motor &ioutput,
                                                const QLength &idiameter,
                                                const AbstractMotor::GearsetRatioPair &ipair) {
  return withOutput(std::make_shared<Motor>(ioutput), idiameter, ipair);
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const MotorGroup &ioutput,
                                                const QLength &idiameter,
                                                const AbstractMotor::GearsetRatioPair &ipair) {
  return withOutput(std::make_shared<MotorGroup>(ioutput), idiameter, ipair);
}

AsyncMotionProfileControllerBuilder &AsyncMotionProfileControllerBuilder::withOutput(
  const std::shared_ptr<ControllerOutput<double>> &ioutput,
  const QLength &idiameter,
  const AbstractMotor::GearsetRatioPair &ipair) {
  hasOutput = true;
  hasModel = false;
  output = ioutput;
  diameter = idiameter;
  pair = ipair;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const ChassisController &icontroller) {
  return withOutput(icontroller.getChassisModel(),
                    icontroller.getChassisScales(),
                    icontroller.getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilder &AsyncMotionProfileControllerBuilder::withOutput(
  const std::shared_ptr<ChassisController> &icontroller) {
  return withOutput(icontroller->getChassisModel(),
                    icontroller->getChassisScales(),
                    icontroller->getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const std::shared_ptr<ChassisModel> &imodel,
                                                const ChassisScales &iscales,
                                                const AbstractMotor::GearsetRatioPair &ipair) {
  hasOutput = false;
  hasModel = true;
  model = imodel;
  scales = iscales;
  pair = ipair;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withLimits(const PathfinderLimits &ilimits) {
  hasLimits = true;
  limits = ilimits;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
  timeUtilFactory = itimeUtilFactory;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
  controllerLogger = ilogger;
  return *this;
}

std::shared_ptr<AsyncLinearMotionProfileController>
AsyncMotionProfileControllerBuilder::buildLinearMotionProfileController() {
  if (!hasOutput) {
    logger->error("AsyncMotionProfileControllerBuilder: No output given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No output given.");
  }

  if (!hasLimits) {
    logger->error("AsyncMotionProfileControllerBuilder: No limits given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No limits given.");
  }

  auto out = std::make_shared<AsyncLinearMotionProfileController>(
    timeUtilFactory.create(), limits, output, diameter, pair, controllerLogger);
  out->startThread();
  out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  return out;
}

std::shared_ptr<AsyncMotionProfileController>
AsyncMotionProfileControllerBuilder::buildMotionProfileController() {
  if (!hasModel) {
    logger->error("AsyncMotionProfileControllerBuilder: No model given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No model given.");
  }

  if (!hasLimits) {
    logger->error("AsyncMotionProfileControllerBuilder: No limits given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No limits given.");
  }

  auto out = std::make_shared<AsyncMotionProfileController>(
    timeUtilFactory.create(), limits, model, scales, pair, controllerLogger);
  out->startThread();
  out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  return out;
}
} // namespace okapi
