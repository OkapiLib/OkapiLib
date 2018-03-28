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
#include "okapi/control/iterative/posPidController.hpp"
#include "okapi/device/motor/abstractMotor.hpp"

namespace okapi {
class AsyncPosPIDController;

class AsyncPosPIDControllerParams : public AsyncPositionControllerParams {
  public:
  AsyncPosPIDControllerParams(ControllerInput &iinput, ControllerOutput &ioutput,
                              const PosPIDControllerParams &iparams);

  ControllerInput &input;
  ControllerOutput &output;
  const PosPIDControllerParams &params;
};

class AsyncPosPIDController : public AsyncPositionController {
  public:
  AsyncPosPIDController(ControllerInput &iinput, ControllerOutput &ioutput,
                        const PosPIDControllerParams &iparams);

  AsyncPosPIDController(ControllerInput &iinput, ControllerOutput &ioutput, const double ikP,
                        const double ikI, const double ikD, const double ikBias = 0);

  virtual ~AsyncPosPIDController();

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
   * Set time between loops in ms. Default does nothing.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const uint32_t isampleTime) override;

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
   * Change whether the controll is off or on. Default does nothing.
   */
  virtual void flipDisable() override;

  protected:
  ControllerInput &input;
  ControllerOutput &output;
  PosPIDController controller;
  uint32_t prevTime = 0;
  pros::Task task;

  static void trampoline(void *context);
  void step();
};
} // namespace okapi

#endif
