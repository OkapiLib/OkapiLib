/**
 * Based on the Arduino PID controller: https://github.com/br3ttb/Arduino-PID-Library
 *
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_PID_HPP_
#define _OKAPI_PID_HPP_

#include "okapi/control/iterative/iterativePositionController.hpp"

namespace okapi {
class PosPIDControllerParams : public IterativePositionControllerParams {
  public:
  PosPIDControllerParams(const double ikP, const double ikI, const double ikD,
                         const double ikBias = 0);

  virtual ~PosPIDControllerParams();

  const double kP, kI, kD, kBias;
};

class PosPIDController : public IterativePositionController {
  public:
  /**
   * PID controller.
   *
   * @param ikP proportional gain
   * @param ikI integral gain
   * @param ikD derivative gain
   * @param ikBias controller bias (constant offset added to the output)
   */
  PosPIDController(const double ikP, const double ikI, const double ikD, const double ikBias = 0);

  /**
   * PID controller.
   *
   * @param params PosPIDControllerParams
   */
  PosPIDController(const PosPIDControllerParams &params);

  virtual ~PosPIDController();

  /**
   * Do one iteration of the controller. Returns the reading in the range [-127, 127] unless the
   * bounds have been changed with setOutputLimits().
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual double step(const double inewReading) override;

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) override;

  /**
   * Returns the last calculated output of the controller. Output is in the range [-127, 127]
   * unless the bounds have been changed with setOutputLimits().
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
   * Set controller gains.
   *
   * @param ikP proportional gain
   * @param ikI integral gain
   * @param ikD derivative gain
   * @param ikBias bias (constant offset added to the output)
   */
  virtual void setGains(const double ikP, const double ikI, const double ikD,
                        const double ikBias = 0);

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const uint32_t isampleTime) override;

  /**
   * Set controller output bounds. Default bounds are [-127, 127].
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin) override;

  /**
   * Set integrator bounds. Default bounds are [-127, 127];
   *
   * @param imax max integrator value
   * @param imin min integrator value
   */
  virtual void setIntegralLimits(double imax, double imin);

  /**
   * Resets the controller so it can start from 0 again properly. Keeps gains and limits from
   * before.
   */
  virtual void reset() override;

  /**
   * Set whether the integrator should be reset when error is 0 or changes sign.
   *
   * @param iresetOnZero true to reset
   */
  virtual void setIntegratorReset(bool iresetOnZero);

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

  protected:
  double kP, kI, kD, kBias;
  uint32_t lastTime = 0;
  uint32_t sampleTime = 0;
  double target = 0;
  double lastReading = 0;
  double error = 0;
  const double errorScale = 4096; // 12 bit ADC scale, good enough for most things
  double lastError = 0;
  double integral = 0;
  double integralMax = 1;
  double integralMin = -1;
  double derivative = 0;
  double output = 0;
  double outputMax = 0;
  double outputMin = 0;
  bool shouldResetOnCross = true;
  bool isOn = true;
};
} // namespace okapi

#endif
