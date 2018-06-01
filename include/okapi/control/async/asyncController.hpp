/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCCONTROLLER_HPP_
#define _OKAPI_ASYNCCONTROLLER_HPP_

#include "api.h"
#include "okapi/control/closedLoopController.hpp"
#include "okapi/units/QTime.hpp"

namespace okapi {
class AsyncControllerArgs {
  public:
  virtual ~AsyncControllerArgs();
};

/**
 * Closed-loop controller that steps on its own in another thread and automatically writes to the
 * output.
 */
class AsyncController : public ClosedLoopController {
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
  virtual void setSampleTime(const QTime isampleTime);

  /**
   * Set controller output bounds. Default does nothing.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin);
};
} // namespace okapi

#endif
