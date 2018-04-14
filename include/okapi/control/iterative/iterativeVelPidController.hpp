/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ITERATIVEVELPIDCONTROLLER_HPP_
#define _OKAPI_ITERATIVEVELPIDCONTROLLER_HPP_

#include "okapi/control/iterative/iterativeVelocityController.hpp"
#include "okapi/control/util/settledUtil.hpp"
#include "okapi/filter/velMath.hpp"

namespace okapi {
class IterativeVelPIDControllerArgs : public IterativeVelocityControllerArgs {
  public:
  IterativeVelPIDControllerArgs(const double ikP, const double ikD);

  IterativeVelPIDControllerArgs(const double ikP, const double ikD, const VelMathArgs &iparams);

  const double kP, kD;
  const VelMathArgs params{1800};
};

class IterativeVelPIDController : public IterativeVelocityController {
  public:
  /**
   * Velocity PID controller.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   */
  IterativeVelPIDController(const double ikP, const double ikD);

  /**
   * Velocity PID controller.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   */
  IterativeVelPIDController(const double ikP, const double ikD, const VelMathArgs &iparams);

  /**
   * Velocity PID controller.
   *
   * @param params VelPIDControllerArgs
   */
  IterativeVelPIDController(const IterativeVelPIDControllerArgs &params);

  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual double step(const double inewReading) override;

  /**
   * Sets the target for the controller.
   *
   * @param itarget new target velocity
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last calculated output of the controller.
   */
  virtual double getOutput() const override;

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const override;

  /**
   * Returns the last derivative (change in error) of the controller.
   */
  virtual double getDerivative() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  virtual bool isSettled() override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const uint32_t isampleTime) override;

  /**
   * Set controller output bounds.
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
   * Change whether the controll is off or on.
   */
  virtual void flipDisable() override;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual uint32_t getSampleTime() const override;

  /**
   * Do one iteration of velocity calculation.
   *
   * @param inewReading new measurement
   * @return filtered velocity
   */
  virtual double stepVel(const double inewReading);

  /**
   * Set controller gains.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikBias controller bias
   */
  virtual void setGains(const double ikP, const double ikD);

  /**
   * Sets the number of encoder ticks per revolution. Default is 1800.
   *
   * @param tpr number of measured units per revolution
   */
  virtual void setTicksPerRev(const double tpr);

  /**
   * Returns the current velocity.
   */
  virtual double getVel() const;

  protected:
  double kP, kD;
  uint32_t lastTime = 0;
  uint32_t sampleTime = 10;
  double error = 0;
  double lastError = 0;
  double derivative = 0;
  double target = 0;
  double output = 0;
  double outputMax = 1;
  double outputMin = -1;
  bool isOn = true;
  VelMath velMath{1800};
  SettledUtil settledUtil;
};
} // namespace okapi

#endif
