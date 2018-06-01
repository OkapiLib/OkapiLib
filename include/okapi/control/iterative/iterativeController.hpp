/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ITERATIVECONTROLLER_HPP_
#define _OKAPI_ITERATIVECONTROLLER_HPP_

#include "api.h"
#include "okapi/control/closedLoopController.hpp"
#include "okapi/units/QTime.hpp"

namespace okapi {
class IterativeControllerArgs {
  public:
  virtual ~IterativeControllerArgs();
};

/**
 * Closed-loop controller that steps iteratively using the step method below.
 */
class IterativeController : public ClosedLoopController {
  public:
  /**
   * Do one iteration of the controller. Outputs in the range [-1, 1]
   *
   * @param inewReading new measurement
   * @return controller [-1, 1]
   */
  virtual double step(const double ireading) = 0;

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
  virtual void setSampleTime(const QTime isampleTime);

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin);

  /**
   * Get the last set sample time. Default is 10 milliseconds.
   *
   * @return sample time
   */
  virtual QTime getSampleTime() const;
};
} // namespace okapi

#endif
