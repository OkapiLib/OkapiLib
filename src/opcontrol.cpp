#include "api.h"

#include "okapi/chassis/chassisController.hpp"
#include "okapi/chassis/chassisModel.hpp"
#include "okapi/chassis/odomChassisController.hpp"

#include "okapi/device/adiButton.hpp"
#include "okapi/device/controllerButton.hpp"
#include "okapi/device/motor.hpp"
#include "okapi/odometry/odomMath.hpp"

void opcontrol() {
  while (true) {
    okapi::ADIButton btn(2);
    btn.isPressed();
    btn.edge();
    btn.risingEdge();
    btn.fallingEdge();

    okapi::ControllerButton btn2(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A);

    {
      using namespace okapi;

      pros::Motor mtr = 1_m;
      pros::Motor r_mtr = 2_rm;

      SkidSteerModel<2> model1({2_m, 3_m, 4_m, 5_m}, //Left motors: 2 & 3, right motors: 4 & 5
                    QuadEncoder(1, 2, true), //Left encoder (reversed)
                    QuadEncoder(3, 4)); //Right encoder
      
      XDriveModel<1> model2({2_m, 3_m, 4_m, 5_m}, //Motors are ordered counter-clockwise from the top left
                        QuadEncoder(1, 2, true), //Top left encoder (reversed)
                        QuadEncoder(3, 4)); //Top right encoder
      
      ChassisControllerPid controller1(
        SkidSteerModelParams<2>(
          {2_m, 3_m, 4_m, 5_m},
          QuadEncoder(1, 2, true),
          QuadEncoder(3, 4)),
        PidParams(0, 0, 0),
        PidParams(0, 0, 0));

      ChassisControllerPid controller2(
        XDriveModelParams<1>(
          {2_m, 3_m, 4_m, 5_m},
          QuadEncoder(1, 2, true),
          QuadEncoder(3, 4)),
        PidParams(0, 0, 0),
        PidParams(0, 0, 0));
      
      OdomChassisControllerPid controller3(
        OdomParams(
          SkidSteerModelParams<2>(
            {2_m, 3_m, 4_m, 5_m},
            QuadEncoder(1, 2, true),
            QuadEncoder(3, 4)),
          0,
          0
        ),
        PidParams(0, 0, 0),
        PidParams(0, 0, 0));
      
      OdomChassisControllerPid controller4(
        OdomParams(
          XDriveModelParams<1>(
            {2_m, 3_m, 4_m, 5_m},
            QuadEncoder(1, 2, true),
            QuadEncoder(3, 4)),
          0,
          0
        ),
        PidParams(0, 0, 0),
        PidParams(0, 0, 0));
    }
  }
}
