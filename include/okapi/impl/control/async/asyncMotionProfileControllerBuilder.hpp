/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
class AsyncMotionProfileControllerBuilder {
  public:
  /**
   * A builder that creates async motion profile controllers. Use this to build an
   * AsyncMotionProfileController or an AsyncLinearMotionProfileController.
   */
  AsyncMotionProfileControllerBuilder();

  /**
   * Sets the output.
   *
   * @param ioutput The output.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withOutput(const Motor &ioutput);

  /**
   * Sets the output.
   *
   * @param ioutput The output.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withOutput(const MotorGroup &ioutput);

  /**
   * Sets the output.
   *
   * @param ioutput The output.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &
  withOutput(const std::shared_ptr<ControllerOutput<double>> &ioutput);

  /**
   * Sets the output.
   *
   * @param icontroller The chassis controller to use.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withOutput(const ChassisController &icontroller);

  /**
   * Sets the output.
   *
   * @param icontroller The chassis controller to use.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &
  withOutput(const std::shared_ptr<ChassisController> &icontroller);

  /**
   * Sets the output.
   *
   * @param imodel The chassis model to use.
   * @param iscales The chassis dimensions.
   * @param ipair The gearset.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withOutput(const std::shared_ptr<ChassisModel> &imodel,
                                                  const ChassisScales &iscales,
                                                  const AbstractMotor::GearsetRatioPair &ipair);

  /**
   * Sets the limits.
   *
   * @param ilimits The limits.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withLimits(const PathfinderLimits &ilimits);

  /**
   * Sets the TimeUtilFactory used when building the controller. The default is the static
   * TimeUtilFactory.
   *
   * @param itimeUtilFactory The TimeUtilFactory.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilder &withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory);

  /**
   * Builds the AsyncLinearMotionProfileController.
   *
   * @return A fully built AsyncLinearMotionProfileController.
   */
  std::shared_ptr<AsyncLinearMotionProfileController> buildLinearMotionProfileController();

  /**
   * Builds the AsyncMotionProfileController.
   *
   * @return A fully built AsyncMotionProfileController.
   */
  std::shared_ptr<AsyncMotionProfileController> buildMotionProfileController();

  private:
  Logger *logger;

  bool hasLimits{false};
  PathfinderLimits limits;

  bool isLinear{false}; // Whether to build a linear controller or not
  bool hasOutput{false};
  std::shared_ptr<ControllerOutput<double>> output;

  std::shared_ptr<ChassisModel> model;
  ChassisScales scales{1, 1};
  AbstractMotor::GearsetRatioPair pair{AbstractMotor::gearset::invalid};

  TimeUtilFactory timeUtilFactory = TimeUtilFactory();

  void validateBuilder();
};
} // namespace okapi
