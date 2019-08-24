/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/units/QLength.hpp"

namespace okapi {
struct Point2d {
  QLength x{0_m};
  QLength y{0_m};

  /**
   * Computes the value of this point in the given StateMode.
   *
   * @param imode The StateMode.
   * @return This point in the StateMode.
   */
  Point2d inMode(const StateMode &imode) const {
    if (imode == StateMode::FRAME_TRANSFORMATION) {
      return *this;
    } else {
      return {y, x};
    }
  }
};
} // namespace okapi
