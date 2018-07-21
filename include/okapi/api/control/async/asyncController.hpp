/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCCONTROLLER_HPP_
#define _OKAPI_ASYNCCONTROLLER_HPP_

#include "okapi/api/control/closedLoopController.hpp"
#include "okapi/api/units/QTime.hpp"

namespace okapi {
class AsyncControllerArgs {
  public:
  virtual ~AsyncControllerArgs();
};

/**
 * Closed-loop controller that steps on its own in another thread and automatically writes to the
 * output.
 */
class AsyncController : public ClosedLoopController<double> {
  public:
  /**
   * Returns the last calculated output of the controller. Default is 0.
   */
  virtual double getOutput() const;

  /**
   * Set time between loops in ms. Default does nothing.
   *
   * @param isampleTime time between loops
   */
  virtual void setSampleTime(QTime isampleTime);

  /**
   * Set controller output bounds. Default does nothing.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin);

  /**
   * Blocks the current task until the controller has settled. Determining what settling means is
   * implementation-dependent.
   */
  virtual void waitUntilSettled() = 0;
};
} // namespace okapi

#endif
