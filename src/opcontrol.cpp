#include "api.h"
#include "okapi/device/adiButton.hpp"
#include "okapi/device/controllerButton.hpp"
#include "okapi/device/motor.hpp"
#include "okapi/chassis/chassisModel.hpp"

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

      okapi::SkidSteerModel<2> model({2_m, 3_m, 4_m, 5_m}, //Left motors: 2 & 3, right motors: 4 & 5
                    okapi::QuadEncoder(1, 2, true), //Left encoder (reversed)
                    okapi::QuadEncoder(3, 4)); //Right encoder
    }
  }
}
