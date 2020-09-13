/*
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
AsyncMotionProfileControllerBuilder::withOutput(ChassisController &icontroller) {
  return withOutput(
    icontroller.getModel(), icontroller.getChassisScales(), icontroller.getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilder &AsyncMotionProfileControllerBuilder::withOutput(
  const std::shared_ptr<ChassisController> &icontroller) {
  return withOutput(
    icontroller->getModel(), icontroller->getChassisScales(), icontroller->getGearsetRatioPair());
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

AsyncMotionProfileControllerBuilder &AsyncMotionProfileControllerBuilder::parentedToCurrentTask() {
  isParentedToCurrentTask = true;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::notParentedToCurrentTask() {
  isParentedToCurrentTask = false;
  return *this;
}

std::shared_ptr<AsyncLinearMotionProfileController>
AsyncMotionProfileControllerBuilder::buildLinearMotionProfileController() {
  if (!hasOutput) {
    std::string msg("AsyncMotionProfileControllerBuilder: No output given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilder: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  auto out = std::make_shared<AsyncLinearMotionProfileController>(
    timeUtilFactory.create(), limits, output, diameter, pair, controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<AsyncMotionProfileController>
AsyncMotionProfileControllerBuilder::buildMotionProfileController() {
  if (!hasModel) {
    std::string msg("AsyncMotionProfileControllerBuilder: No model given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilder: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  auto out = std::make_shared<AsyncMotionProfileController>(
    timeUtilFactory.create(), limits, model, scales, pair, controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}
} // namespace okapi
