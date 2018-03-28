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
  const VelMathParams params{360};
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
   * Set the gains for the DemaFilter. Defaults are 0.19 and 0.0526,
   * respectively.
   *
   * @param alpha alpha gain
   * @param beta beta gain
   */
  virtual void setFilterGains(const double alpha, const double beta);

  /**
   * Set the number of measurements per revolution. Default is 360.
   *
   * @param tpr number of measured units per revolution
   */
  virtual void setTicksPerRev(const double tpr);

  /**
   * Get the current velocity.
   */
  virtual double getVel() const;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual uint32_t getSampleTime() const override;

  protected:
  double kP, kD;
  uint32_t lastTime = 0;
  uint32_t sampleTime = 15;
  double error = 0;
  double lastError = 0;
  double derivative = 0;
  double target = 0;
  double output = 0;
  double outputMax = 1;
  double outputMin = -1;
  bool isOn = true;
  VelMath velMath{360};
};
} // namespace okapi

#endif
