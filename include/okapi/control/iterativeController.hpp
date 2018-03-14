/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ITERATIVECONTROLLER_HPP_
#define _OKAPI_ITERATIVECONTROLLER_HPP_

#include "api.h"

namespace okapi {
class IterativeControllerParams {};

/**
 * Closed-loop controller that steps iteratively using the step method below.
 */
class IterativeController {
  public:
  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual double step(const double ireading) = 0;

  /**
   * Sets the target for the controller.
   */
  virtual void setTarget(const double itarget) = 0;

  /**
   * Returns the last calculated output of the controller.
   */
  virtual double getOutput() const = 0;

  /**
   * Returns the last error of the controller.
   */
  virtual double getError() const = 0;

  /**
   * Set time between loops in ms.
   *
   * @param isampleTime time between loops in ms
   */
  virtual void setSampleTime(const uint32_t isampleTime) {
  }

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(double imax, double imin) {
  }

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  virtual void reset() {
  }

  /**
   * Change whether the controll is off or on.
   */
  virtual void flipDisable() {
  }
};
} // namespace okapi

#endif
