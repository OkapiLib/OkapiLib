/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ITERATIVECONTROLLER_HPP_
#define _OKAPI_ITERATIVECONTROLLER_HPP_

#include "okapi/api/control/closedLoopController.hpp"
#include "okapi/api/units/QTime.hpp"

namespace okapi {
class IterativeControllerArgs {
  public:
  virtual ~IterativeControllerArgs();
};

/**
 * Closed-loop controller that steps iteratively using the step method below.
 */
class IterativeController : public ClosedLoopController<double, double> {
  public:
  /**
   * Do one iteration of the controller. Outputs in the range [-1, 1]
   *
   * @param inewReading new measurement
   * @return controller [-1, 1]
   */
  virtual double step(double ireading) = 0;

  /**
   * Returns the last calculated output of the controller.
   */
  virtual double getOutput() const = 0;

  /**
   * Returns the last derivative (change in error) of the controller.
   */
  virtual double getDerivative() const = 0;

  /**
   * Set time between loops.
   *
   * @param isampleTime time between loops
   */
  virtual void setSampleTime(QTime isampleTime) = 0;

  /**
   * Get the last set sample time.
   *
   * @return sample time
   */
  virtual QTime getSampleTime() const = 0;

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin) = 0;
};
} // namespace okapi

#endif
