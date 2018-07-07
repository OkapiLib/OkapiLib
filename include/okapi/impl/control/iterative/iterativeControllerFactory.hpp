/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPILIB_ITERATIVECONTROLLERFACTORY_HPP_
#define _OKAPILIB_ITERATIVECONTROLLERFACTORY_HPP_

#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/control/iterative/iterativePosPidController.hpp"
#include "okapi/impl/control/iterative/iterativeVelPidController.hpp"

namespace okapi {
class IterativeControllerFactory {
  public:
  /**
   * PID controller.
   *
   * @param ikP proportional gain
   * @param ikI integral gain
   * @param ikD derivative gain
   * @param ikBias controller bias (constant offset added to the output)
   */
  static IterativePosPIDController posPID(double ikP, double ikI, double ikD, double ikBias = 0);

  /**
   * Velocity PID controller.
   *
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   */
  static IterativeVelPIDController velPID(double ikP, double ikD, double ikF = 0,
                                          const VelMathArgs &iparams = VelMathArgs(imev5TPR));
};
} // namespace okapi

#endif
