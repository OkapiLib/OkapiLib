#include "api.h"
#include "okapi/device/adiButton.hpp"
#include "okapi/device/controllerButton.hpp"
#include "okapi/device/motor.hpp"

void opcontrol() {
  while (true) {
    okapi::ADIButton btn(2);
    btn.isPressed();
    btn.edge();
    btn.risingEdge();
    btn.fallingEdge();

    okapi::ControllerButton btn2(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A);

    {
      using namespace okapi::literals;

      pros::Motor mtr = 1_m;
      pros::Motor r_mtr = 2_rm;
    }
  }
}
