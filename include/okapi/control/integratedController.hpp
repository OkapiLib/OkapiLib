/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_INTEGRATEDCONTROLLER_HPP_
#define _OKAPI_INTEGRATEDCONTROLLER_HPP_

#include "okapi/device/abstractMotor.hpp"
#include "okapi/device/integratedEncoder.hpp"

namespace okapi {
class IntegratedControllerParams {
  public:
  IntegratedControllerParams(const AbstractMotor &imotor);

  const AbstractMotor &motor;
};

/**
 * Closed-loop controller that uses the V5 motor's onboard control to move.
 */
class IntegratedController {
  public:
  IntegratedController(const AbstractMotor &imotor);

  IntegratedController(const IntegratedControllerParams &iparams);

  virtual ~IntegratedController();

  int32_t moveAbsolute(const double position, const int32_t velocity) const;

  int32_t moveRelative(const double position, const int32_t velocity) const;

  int32_t moveVelocity(const int16_t velocity) const;

  int32_t moveVoltage(const int16_t voltage) const;

  private:
  const AbstractMotor &motor;
};
} // namespace okapi

#endif
