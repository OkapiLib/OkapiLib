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

  virtual double runUntilSettled(const double itarget, AsyncController &icontroller);

  virtual double runUntilSettled(const double itarget, IterativeController &icontroller,
                                 ControllerOutput &ioutput);

  virtual double runUntilAtTarget(const double itarget, AsyncController &icontroller);

  virtual double runUntilAtTarget(const double itarget, IterativeController &icontroller,
                                  ControllerOutput &ioutput);
};
}

#endif
