/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncMotionProfileControllerBuilder.hpp"

namespace okapi {
AsyncMotionProfileControllerBuilder::AsyncMotionProfileControllerBuilder()
  : logger(Logger::instance()) {
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const Motor &ioutput) {
  return withOutput(ioutput);
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const MotorGroup &ioutput) {
  return withOutput(ioutput);
}

AsyncMotionProfileControllerBuilder &AsyncMotionProfileControllerBuilder::withOutput(
  const std::shared_ptr<ControllerOutput<double>> &ioutput) {
  hasOutput = true;
  isLinear = true;
  output = ioutput;
  return *this;
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const ChassisController &icontroller) {
  return withOutput(icontroller.getChassisModel(),
                    icontroller.getChassisScales(),
                    icontroller.getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilder &
AsyncMotionProfileControllerBuilder::withOutput(const std::shared_ptr<ChassisModel> &imodel,
                                                const ChassisScales &iscales,
                                                const AbstractMotor::GearsetRatioPair &ipair) {
  hasOutput = true;
  isLinear = false;
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

std::shared_ptr<AsyncLinearMotionProfileController>
AsyncMotionProfileControllerBuilder::buildLinearMotionProfileController() {
  validateBuilder();

  auto out =
    std::make_shared<AsyncLinearMotionProfileController>(timeUtilFactory.create(), limits, output);
  out->startThread();
  return out;
}

std::shared_ptr<AsyncMotionProfileController>
AsyncMotionProfileControllerBuilder::buildMotionProfileController() {
  validateBuilder();

  auto out = std::make_shared<AsyncMotionProfileController>(
    timeUtilFactory.create(), limits, model, scales, pair);
  out->startThread();
  return out;
}

void AsyncMotionProfileControllerBuilder::validateBuilder() {
  if (!hasOutput) {
    logger->error("AsyncMotionProfileControllerBuilder: No output given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No output given.");
  }

  if (!hasLimits) {
    logger->error("AsyncMotionProfileControllerBuilder: No limits given.");
    throw std::runtime_error("AsyncMotionProfileControllerBuilder: No limits given.");
  }
}
} // namespace okapi
