/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/device/adiButton.hpp"

namespace okapi {
ADIButton::ADIButton(const uint8_t iport, const bool iinverted)
  : btn(iport), port(iport), inverted(iinverted), wasPressedLast(false) {
}

ADIButton::~ADIButton() = default;

bool ADIButton::isPressed() {
  wasPressedLast = currentlyPressed();
  return wasPressedLast;
}

bool ADIButton::changed() {
  const bool pressed = currentlyPressed();
  const bool out = pressed ^ wasPressedLast;
  wasPressedLast = pressed;
  return out;
}

bool ADIButton::changedToPressed() {
  return changed() && wasPressedLast;
}

bool ADIButton::changedToReleased() {
  return changed() && !wasPressedLast;
}

bool ADIButton::currentlyPressed() {
  const int32_t state = btn.value_get();
  const bool pressed = state != 0 && state != PROS_ERR;
  return inverted ? !pressed : pressed;
}
} // namespace okapi