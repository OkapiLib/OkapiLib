#include "api.h"
#include "okapi/device/button.hpp"

void opcontrol() {
  while (true) {
    okapi::Button btn(2);
    btn.isPressed();
    btn.edge();
    btn.risingEdge();
    btn.fallingEdge();
  }
}
