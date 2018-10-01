#include "okapi/api/device/button/buttonBase.hpp"

namespace okapi {
ButtonBase::ButtonBase() = default;

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
