/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/device/integratedEncoder.hpp"

namespace okapi {
IntegratedEncoder::IntegratedEncoder(const pros::Motor &imotor) : motor(imotor) {
}

int32_t IntegratedEncoder::get() const {
}

int32_t IntegratedEncoder::reset() const {
}
} // namespace okapi
