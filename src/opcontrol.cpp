#include "api.h"
#include "okapi/device/button.hpp"
#include "okapi/device/motor.hpp"

void opcontrol() {
  while (true) {
    okapi::Button btn(2);
    btn.isPressed();
    btn.edge();
    btn.risingEdge();
    btn.fallingEdge();

    // {
    //   using namespace okapi::literals;

    //   pros::Motor mtr = 1_m;
    //   pros::Motor r_mtr = 2_rm;
    // }
  }
}
