/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_SETTLEDUTIL_HPP_
#define _OKAPI_SETTLEDUTIL_HPP_

#include "okapi/units/QTime.hpp"
#include "okapi/util/timer.hpp"

namespace okapi {
class SettledUtil {
  public:
  /**
   * A utility class to determine if a control loop has settled based on error. A control loop is
   * settled if the error is within atTargetError for atTargetTime.
   *
   * @param iatTargetError minimum error to be considered settled
   * @param iatTargetDerivative minimum error derivative to be considered settled
   * @param iatTargetTime minimum time within atTargetError to be considered settled
   */
  SettledUtil(const double iatTargetError = 50, const double iatTargetDerivative = 5,
              const QTime iatTargetTime = 250_ms);

  virtual ~SettledUtil();

  /**
   * Returns whether the controller is settled.
   *
   * @param ierror current error
   * @return whether the controller is settled
   */
  virtual bool isSettled(const double ierror);

  /**
   * Resets the "at target" timer.
   */
  virtual void reset();

  protected:
  double atTargetError = 50;
  double atTargetDerivative = 5;
  QTime atTargetTime = 250_ms;
  Timer atTargetTimer;
  double lastError = 0;
};
} // namespace okapi

#endif
