/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_
#define _OKAPI_ASYNCPOSPIDCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/async/asyncPositionController.hpp"
#include "okapi/control/controllerInput.hpp"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/control/iterative/iterativePosPidController.hpp"
#include "okapi/device/motor/abstractMotor.hpp"
#include <memory>

namespace okapi {
class AsyncPosPIDController;

class AsyncPosPIDControllerArgs : public AsyncPositionControllerArgs {
  public:
  AsyncPosPIDControllerArgs(std::shared_ptr<ControllerInput> iinput,
                            std::shared_ptr<ControllerOutput> ioutput,
                            const IterativePosPIDControllerArgs &iparams);

  std::shared_ptr<ControllerInput> input;
  std::shared_ptr<ControllerOutput> output;
  const IterativePosPIDControllerArgs &params;
};

class AsyncPosPIDController : public AsyncPositionController {
  public:
  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput,
                        const IterativePosPIDControllerArgs &iparams);

  AsyncPosPIDController(std::shared_ptr<ControllerInput> iinput,
                        std::shared_ptr<ControllerOutput> ioutput, const double ikP,
                        const double ikI, const double ikD, const double ikBias = 0);

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last calculated output of the controller. Default is 0.
   */
  virtual double getOutput() const override;

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  virtual bool isSettled() override;

  /**
   * Set time between loops in ms. Default does nothing.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const std::uint32_t isampleTime) override;

  /**
   * Set controller output bounds. Default does nothing.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin) override;

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  virtual void reset() override;

  /**
   * Changes whether the controll is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  virtual void flipDisable() override;

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  virtual void flipDisable(const bool iisDisabled) override;

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  virtual bool isDisabled() const override;

  protected:
  std::shared_ptr<ControllerInput> input;
  std::shared_ptr<ControllerOutput> output;
  IterativePosPIDController controller;
  pros::Task task;

  static void trampoline(void *context);
  void step();
};
} // namespace okapi

#endif
