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
/**
 * Closed-loop controller that steps iteratively using the step method below.
 *
 * ControllerOutput::controllerSet() should set the controller's target to the input scaled by the
 * output bounds.
 */
template <typename Input, typename Output>
class IterativeController : public ClosedLoopController<Input, Output> {
  public:
  /**
   * Do one iteration of the controller.
   *
   * @param inewReading new measurement
   * @return controller output
   */
  virtual Output step(Input ireading) = 0;

  /**
   * Returns the last calculated output of the controller.
   */
  virtual Output getOutput() const = 0;

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  virtual void setOutputLimits(Output imax, Output imin) = 0;

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  virtual Output getMaxOutput() = 0;

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  virtual Output getMinOutput() = 0;

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
};
} // namespace okapi

#endif
