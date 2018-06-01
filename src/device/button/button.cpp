/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/device/button/button.hpp"

namespace okapi {
AbstractButton::AbstractButton(const bool iinverted) : inverted(iinverted) {
}

AbstractButton::~AbstractButton() = default;

bool AbstractButton::isPressed() {
  wasPressedLast = currentlyPressed();
  return wasPressedLast;
}

bool AbstractButton::changed() {
  const bool pressed = currentlyPressed();
  const bool out = pressed ^ wasPressedLast;
  wasPressedLast = pressed;
  return out;
}

bool AbstractButton::changedToPressed() {
  return changed() && wasPressedLast;
}

bool AbstractButton::changedToReleased() {
  return changed() && !wasPressedLast;
}
} // namespace okapi
