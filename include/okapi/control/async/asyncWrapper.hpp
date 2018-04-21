/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCWRAPPER_HPP_
#define _OKAPI_ASYNCWRAPPER_HPP_

#include "okapi/control/async/asyncController.hpp"
#include "okapi/control/controllerInput.hpp"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/control/iterative/iterativeController.hpp"
#include <memory>

namespace okapi {
class AsyncWrapper : virtual public AsyncController {
  public:
  /**
   * A wrapper class that transforms an IterativeController into an AsyncController by running it in
   * another task. The input controller will act like an AsyncController.
   *
   * @param iinput controller input, passed to the IterativeController
   * @param ioutput controller output, written to from the IterativeController
   * @param icontroller the controller to use
   */
  AsyncWrapper(std::shared_ptr<ControllerInput> iinput, std::shared_ptr<ControllerOutput> ioutput,
               std::unique_ptr<IterativeController> icontroller);

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
   * Changes whether the controller is off or on. Turning the controller on after it was off will
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
  std::unique_ptr<IterativeController> controller;
  pros::Task task;

  static void trampoline(void *context);
  void loop();
};
}

#endif
