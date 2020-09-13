/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/device/button/buttonBase.hpp"

namespace okapi {
ButtonBase::ButtonBase(const bool iinverted) : inverted(iinverted) {
}

bool ButtonBase::isPressed() {
  return currentlyPressed();
}

bool ButtonBase::changed() {
  return changedImpl(wasPressedLast_c);
}

bool ButtonBase::changedToPressed() {
  return changedImpl(wasPressedLast_ctp) && wasPressedLast_ctp;
}

bool ButtonBase::changedToReleased() {
  return changedImpl(wasPressedLast_ctr) && !wasPressedLast_ctr;
}

bool ButtonBase::changedImpl(bool &prevState) {
  const bool pressed = currentlyPressed();
  const bool out = pressed ^ prevState;
  prevState = pressed;
  return out;
}
} // namespace okapi
