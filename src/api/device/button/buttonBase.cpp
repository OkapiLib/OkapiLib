#include "okapi/api/device/button/buttonBase.hpp"

namespace okapi {
ButtonBase::ButtonBase(const bool iinverted) : inverted(iinverted) {
}

bool ButtonBase::isPressed() {
  wasPressedLast = currentlyPressed();
  return wasPressedLast;
}

bool ButtonBase::changed() {
  const bool pressed = currentlyPressed();
  const bool out = pressed ^ wasPressedLast;
  wasPressedLast = pressed;
  return out;
}

bool ButtonBase::changedToPressed() {
  return changed() && wasPressedLast;
}

bool ButtonBase::changedToReleased() {
  return changed() && !wasPressedLast;
}
} // namespace okapi
