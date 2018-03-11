/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_MOTOR_HPP_
#define _OKAPI_MOTOR_HPP_

#include "api.h"

namespace okapi {
  inline namespace literals {
    /**
     * Non-reversed motor.
     **/
    pros::Motor operator"" _m(const unsigned long long iport);

    /**
     * Reversed motor.
     **/
    pros::Motor operator"" _rm(const unsigned long long iport);
  }
}

#endif
