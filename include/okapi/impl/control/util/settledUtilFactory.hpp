/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_SETTLEDUTILFACTORY_HPP_
#define _OKAPI_SETTLEDUTILFACTORY_HPP_

#include "okapi/api/control/util/settledUtil.hpp"
#include <memory>

namespace okapi {
class SettledUtilFactory {
  public:
  /**
   * A utility class to determine if a control loop has settled based on error. A control loop is
   * settled if the error is within atTargetError for atTargetTime.
   *
   * @param iatTargetError minimum error to be considered settled
   * @param iatTargetDerivative minimum error derivative to be considered settled
   * @param iatTargetTime minimum time within atTargetError to be considered settled
   */
  static SettledUtil
  create(double iatTargetError = 50, double iatTargetDerivative = 5, QTime iatTargetTime = 250_ms);
  static std::unique_ptr<SettledUtil> createPtr(double iatTargetError = 50,
                                                double iatTargetDerivative = 5,
                                                QTime iatTargetTime = 250_ms);
};
} // namespace okapi

#endif
