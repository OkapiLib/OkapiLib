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
  virtual ~AsyncController();

  /**
   * Returns the last calculated output of the controller. Default is 0.
   */
  virtual double getOutput() const;

  /**
   * Set time between loops in ms. Default does nothing.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const std::uint32_t isampleTime);

  /**
   * Set controller output bounds. Default does nothing.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin);

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  virtual void reset() = 0;

  /**
   * Change whether the controll is off or on. Default does nothing.
   */
  virtual void flipDisable();
};
} // namespace okapi

#endif
