/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ITERATIVEVELPIDCONTROLLER_HPP_
#define _OKAPI_ITERATIVEVELPIDCONTROLLER_HPP_

#include "okapi/api/control/iterative/iterativeVelocityController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/filter/velMath.hpp"

namespace okapi {
class IterativeVelPIDController : public IterativeVelocityController {
  public:
  /**
   * Velocity PD controller.
   */
  IterativeVelPIDController(double ikP, double ikD, double ikF, std::unique_ptr<VelMath> ivelMath,
                            std::unique_ptr<AbstractTimer> iloopDtTimer,
                            std::unique_ptr<SettledUtil> isettledUtil);

  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  double step(double inewReading) override;

  /**
   * Sets the target for the controller.
   *
   * @param itarget new target velocity
   */
  void setTarget(double itarget) override;

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
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  bool isSettled() override;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops
   */
  void setSampleTime(QTime isampleTime) override;

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
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override;

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  void flipDisable(bool iisDisabled) override;

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  bool isDisabled() const override;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  QTime getSampleTime() const override;

  /**
   * Do one iteration of velocity calculation.
   *
   * @param inewReading new measurement
   * @return filtered velocity
   */
  virtual QAngularSpeed stepVel(double inewReading);

  /**
   * Set controller gains.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikBias controller bias
   */
  virtual void setGains(double ikP, double ikD, double ikF);

  /**
   * Sets the number of encoder ticks per revolution. Default is 1800.
   *
   * @param tpr number of measured units per revolution
   */
  virtual void setTicksPerRev(double tpr);

  /**
   * Returns the current velocity.
   */
  virtual QAngularSpeed getVel() const;

  protected:
  double kP, kD, kF;
  QTime sampleTime = 10_ms;
  double error = 0;
  double derivative = 0;
  double target = 0;
  double output = 0;
  double outputMax = 1;
  double outputMin = -1;
  bool isOn = true;

  std::unique_ptr<VelMath> velMath;
  std::unique_ptr<AbstractTimer> loopDtTimer;
  std::unique_ptr<SettledUtil> settledUtil;
};
} // namespace okapi

#endif
