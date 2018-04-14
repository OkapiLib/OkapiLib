/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CLOSEDLOOPCONTROLLER_HPP_
#define _OKAPI_CLOSEDLOOPCONTROLLER_HPP_

namespace okapi {
/**
 * An abstract closed-loop controller.
 */
class ClosedLoopController {
  public:
  virtual ~ClosedLoopController();

  /**
   * Sets the target for the controller.
   *
   * @param itarget the new target
   */
  virtual void setTarget(const double itarget) = 0;

  /**
   * Returns the last error of the controller.
   *
   * @return the last error
   */
  virtual double getError() const = 0;

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * @return whether the controller is settled
   */
  virtual bool isSettled() = 0;
};
}

#endif
