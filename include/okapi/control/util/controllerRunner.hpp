/**
 * @author Kevin Harrington, Common Wealth Robotics
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLERRUNNER_HPP_
#define _OKAPI_CONTROLLERRUNNER_HPP_

#include "okapi/control/async/asyncController.hpp"
#include "okapi/control/controllerOutput.hpp"
#include "okapi/control/iterative/iterativeController.hpp"

namespace okapi {
class ControllerRunner {
  public:
  ControllerRunner();

  virtual ~ControllerRunner();

  /**
   * Runs the controller until it has settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @return the error when settled
   */
  virtual double runUntilSettled(const double itarget, AsyncController &icontroller);

  /**
   * Runs the controller until it has settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @param ioutput the output to write to
   * @return the error when settled
   */
  virtual double runUntilSettled(const double itarget, IterativeController &icontroller,
                                 ControllerOutput &ioutput);

  /**
   * Runs the controller until it has reached its target, but not necessarily settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @return the error when settled
   */
  virtual double runUntilAtTarget(const double itarget, AsyncController &icontroller);

  /**
   * Runs the controller until it has reached its target, but not necessarily settled.
   *
   * @param itarget the new target
   * @param icontroller the controller to run
   * @param ioutput the output to write to
   * @return the error when settled
   */
  virtual double runUntilAtTarget(const double itarget, IterativeController &icontroller,
                                  ControllerOutput &ioutput);
};
} // namespace okapi

#endif
