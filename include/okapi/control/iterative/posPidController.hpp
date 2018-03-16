/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_PID_HPP_
#define _OKAPI_PID_HPP_

#include "okapi/control/iterative/iterativePositionController.hpp"

namespace okapi {
class PosPIDControllerParams : public IterativePositionControllerParams {
  public:
  PosPIDControllerParams(const double ikP, const double ikI, const double ikD, const double ikBias = 0)
    : kP(ikP), kI(ikI), kD(ikD), kBias(ikBias) {
  }

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
  void setTarget(const double itarget) override;

  /**
   * Returns the last calculated output of the controller. Output is in the range [-127, 127]
   * unless the bounds have been changed with setOutputLimits().
   */
  double getOutput() const override;

  /**
   * Returns the last error of the controller.
   */
  double getError() const override;

  /**
   * Set controller gains.
   *
   * @param ikP proportional gain
   * @param ikI integral gain
   * @param ikD derivative gain
   * @param ikBias bias (constant offset added to the output)
   */
  void setGains(const double ikP, const double ikI, const double ikD, const double ikBias = 0);

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  void setSampleTime(const uint32_t isampleTime) override;

  /**
   * Set controller output bounds. Default bounds are [-127, 127].
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(double imax, double imin) override;

  /**
   * Set integrator bounds. Default bounds are [-127, 127];
   *
   * @param imax max integrator value
   * @param imin min integrator value
   */
  void setIntegralLimits(double imax, double imin);

  /**
   * Resets the controller so it can start from 0 again properly. Keeps gains and limits from
   * before.
   */
  void reset() override;

  /**
   * Set whether the integrator should be reset when error is 0 or changes sign.
   *
   * @param iresetOnZero true to reset
   */
  void setIntegratorReset(bool iresetOnZero);

  /**
   * Change whether the controll is off or on.
   */
  void flipDisable() override;

  protected:
  double kP, kI, kD, kBias;
  uint32_t lastTime, sampleTime;
  double error, lastError;
  double target, lastReading;
  double integral, integralMax, integralMin;
  double output, outputMax, outputMin;
  bool shouldResetOnCross, isOn;
};
} // namespace okapi

#endif
