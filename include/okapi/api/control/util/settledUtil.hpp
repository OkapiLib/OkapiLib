/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/units/QTime.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include <memory>

namespace okapi {
class SettledUtil {
  public:
  /**
   * A utility class to determine if a control loop has settled based on error. A control loop is
   * settled if the error is within atTargetError and atTargetDerivative for atTargetTime.
   *
   * @param iatTargetError minimum error to be considered settled
   * @param iatTargetDerivative minimum error derivative to be considered settled
   * @param iatTargetTime minimum time within atTargetError to be considered settled
   */
  explicit SettledUtil(std::unique_ptr<AbstractTimer> iatTargetTimer,
                       double iatTargetError = 50,
                       double iatTargetDerivative = 5,
                       QTime iatTargetTime = 250_ms);

  virtual ~SettledUtil();

  /**
   * Returns whether the controller is settled.
   *
   * @param ierror current error
   * @return whether the controller is settled
   */
  virtual bool isSettled(double ierror);

  /**
   * Resets the "at target" timer.
   */
  virtual void reset();

  protected:
  double atTargetError = 50;
  double atTargetDerivative = 5;
  QTime atTargetTime = 250_ms;
  std::unique_ptr<AbstractTimer> atTargetTimer;
  double lastError = 0;
};
} // namespace okapi
