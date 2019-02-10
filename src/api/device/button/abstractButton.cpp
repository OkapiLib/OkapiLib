#include "okapi/api/device/button/abstractButton.hpp"

namespace okapi {
AbstractButton::~AbstractButton() = default;

bool AbstractButton::controllerGet() {
  return isPressed();
}
} // namespace okapi
