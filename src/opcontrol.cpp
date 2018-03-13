#include "api.h"

// #include "okapi/chassis/chassisController.hpp"
// #include "okapi/chassis/chassisModel.hpp"
// #include "okapi/chassis/odomChassisController.hpp"

#include "okapi/control/motorGroupController.hpp"
#include "okapi/control/pidController.hpp"
#include "okapi/control/velMath.hpp"
#include "okapi/control/velPidController.hpp"

#include "okapi/device/adiButton.hpp"
#include "okapi/device/controllerButton.hpp"
#include "okapi/device/motor.hpp"
#include "okapi/device/quadEncoder.hpp"

#include "okapi/filter/averageFilter.hpp"
#include "okapi/filter/demaFilter.hpp"
#include "okapi/filter/emaFilter.hpp"

// #include "okapi/odometry/odometry.hpp"
// #include "okapi/odometry/odomMath.hpp"

#include "okapi/util/timer.hpp"

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

      // SkidSteerModel<2> model1({2_m, 3_m, 4_m, 5_m}, //Left motors: 2 & 3, right motors: 4 & 5
      //               QuadEncoder(1, 2, true), //Left encoder (reversed)
      //               QuadEncoder(3, 4)); //Right encoder
      
      // XDriveModel<1> model2({2_m, 3_m, 4_m, 5_m}, //Motors are ordered counter-clockwise from the top left
      //                   QuadEncoder(1, 2, true), //Top left encoder (reversed)
      //                   QuadEncoder(3, 4)); //Top right encoder
      
      // ChassisControllerPID controller1(
      //   SkidSteerModelParams<2>(
      //     {2_m, 3_m, 4_m, 5_m},
      //     QuadEncoder(1, 2, true),
      //     QuadEncoder(3, 4)),
      //   PIDControllerParams(0, 0, 0),
      //   PIDControllerParams(0, 0, 0));

      // ChassisControllerPID controller2(
      //   XDriveModelParams<1>(
      //     {2_m, 3_m, 4_m, 5_m},
      //     QuadEncoder(1, 2, true),
      //     QuadEncoder(3, 4)),
      //   PIDControllerParams(0, 0, 0),
      //   PIDControllerParams(0, 0, 0));
      
      // OdomChassisControllerPID controller3(
      //   OdometryParams(
      //     SkidSteerModelParams<2>(
      //       {2_m, 3_m, 4_m, 5_m},
      //       QuadEncoder(1, 2, true),
      //       QuadEncoder(3, 4)),
      //     0,
      //     0
      //   ),
      //   PIDControllerParams(0, 0, 0),
      //   PIDControllerParams(0, 0, 0));
      
      // OdomChassisControllerPID controller4(
      //   OdometryParams(
      //     XDriveModelParams<1>(
      //       {2_m, 3_m, 4_m, 5_m},
      //       QuadEncoder(1, 2, true),
      //       QuadEncoder(3, 4)),
      //     0,
      //     0
      //   ),
      //   PIDControllerParams(0, 0, 0),
      //   PIDControllerParams(0, 0, 0));

      PIDController pid1(0,0,0);
      MotorGroupController<2> mgController(
        {1_m, 2_m},
        pid1);

      PIDController pid2(0, 0, 0);
      PIDController pid3(0, 0, 0, 0);
      PIDController pid4(PIDControllerParams(0, 0, 0));
      PIDController pid5(PIDControllerParams(0, 0, 0, 0));

      VelMath velMath1(0);
      VelMath velMath2(0, 0);
      VelMath velMath3(0, 0, 0);

      VelPIDController velPid1(0, 0);
      VelPIDController velPid2(VelPIDControllerParams(0, 0));

      QuadEncoder quad1(0, 0);
      QuadEncoder quad2(0, 0, true);

      AverageFilter<1> avgFilt1();

      DemaFilter demaFilt1(0, 0);

      EmaFilter emaFilt1(0, 0);

      // Odometry odom1(
      //   SkidSteerModelParams<2>(
      //     {2_m, 3_m, 4_m, 5_m},
      //     QuadEncoder(1, 2, true),
      //     QuadEncoder(3, 4)),
      //   0,
      //   0);

      Timer timer1();
    }
  }
}
