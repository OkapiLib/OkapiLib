/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_VELPID_HPP_
#define _OKAPI_VELPID_HPP_

#include "okapi/control/iterative/iterativeVelocityController.hpp"
#include "okapi/filter/velMath.hpp"

namespace okapi {
class VelPIDControllerParams : public IterativeVelocityControllerParams {
  public:
  VelPIDControllerParams(const double ikP, const double ikD);

  VelPIDControllerParams(const double ikP, const double ikD, const VelMathParams &iparams);

  virtual ~VelPIDControllerParams();

  const double kP, kD;
  const VelMathParams &params;
};

class VelPIDController : public IterativeVelocityController {
  public:
  /**
   * Velocity PID controller.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   */
  VelPIDController(const double ikP, const double ikD);

  /**
   * Velocity PID controller.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   */
  VelPIDController(const double ikP, const double ikD, const VelMathParams &iparams);

  /**
   * Velocity PID controller.
   *
   * @param params VelPIDControllerParams
   */
  VelPIDController(const VelPIDControllerParams &params);

  virtual ~VelPIDController();

  /**
   * Do one iteration of the controller.
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
   * Returns the last calculated output of the controller.
   */
  double getOutput() const override;

  /**
   * Returns the last error of the controller.
   */
  double getError() const override;

  /**
   * Returns the last derivative (change in error) of the controller.
   */
  double getDerivative() const override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  void setSampleTime(const uint32_t isampleTime) override;

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(double imax, double imin) override;

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  void reset() override;

  /**
   * Change whether the controll is off or on.
   */
  void flipDisable() override;

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
  void setGains(const double ikP, const double ikD);

  /**
   * Set the gains for the DemaFilter. Defaults are 0.19 and 0.0526,
   * respectively.
   *
   * @param alpha alpha gain
   * @param beta beta gain
   */
  void setFilterGains(const double alpha, const double beta);

  /**
   * Set the number of measurements per revolution. Default is 360.
   *
   * @param tpr number of measured units per revolution
   */
  void setTicksPerRev(const double tpr);

  /**
   * Get the current velocity.
   */
  double getVel() const;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  uint32_t getSampleTime() const override;

  private:
  double kP, kD;
  uint32_t lastTime, sampleTime;
  double error, lastError;
  double derivative;
  double target;
  double output, outputMax, outputMin;
  bool isOn;
  VelMath velMath;
};
} // namespace okapi

#endif
