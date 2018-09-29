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
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
class IterativeVelPIDController : public IterativeVelocityController<double, double> {
  public:
  /**
   * Velocity PD controller.
   *
   * @param ikP the proportional gain
   * @param ikD the derivative gain
   * @param ikF the feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   * @param itimeUtil see TimeUtil docs
   * @param iderivativeFilter a filter for filtering the derivative term
   */
  IterativeVelPIDController(
    double ikP,
    double ikD,
    double ikF,
    double ikSF,
    std::unique_ptr<VelMath> ivelMath,
    const TimeUtil &itimeUtil,
    std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * Do one iteration of the controller. Returns the reading in the range [-1, 1] unless the
   * bounds have been changed with setOutputLimits().
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
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller. The range of input values is expected to be [-1, 1].
   *
   * @param ivalue the controller's output in the range [-1, 1]
   */
  void controllerSet(double ivalue) override;

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  double getTarget() override;

  /**
   * Returns the last calculated output of the controller.
   */
  double getOutput() const override;

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  double getMaxOutput() override;

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  double getMinOutput() override;

  /**
   * Returns the last error of the controller.
   */
  double getError() const override;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * If the controller is disabled, this method must return true.
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
   * Set controller output bounds. Default bounds are [-1, 1].
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
   * @param ikF the feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  virtual void setGains(double ikP, double ikD, double ikF, double ikSF);

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
  Logger *logger;
  double kP, kD, kF, kSF;
  QTime sampleTime{10_ms};
  double error{0};
  double derivative{0};
  double target{0};
  double outputSum{0};
  double output{0};
  double outputMax{1};
  double outputMin{-1};
  bool controllerIsDisabled{false};

  std::unique_ptr<VelMath> velMath;
  std::unique_ptr<Filter> derivativeFilter;
  std::unique_ptr<AbstractTimer> loopDtTimer;
  std::unique_ptr<SettledUtil> settledUtil;
};
} // namespace okapi

#endif
