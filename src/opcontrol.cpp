#include "api.h"

#include "okapi/api.hpp"
#include "okapi/test/testRunner.hpp"

using namespace okapi;

void opcontrol() {
  using namespace pros::c;

  task_delay(100);

  // Chassis Controller - lets us drive the robot around with open- or closed-loop control
  okapi::ChassisControllerIntegrated robotChassisController(1_m, 10_m);

  // Joystick to read analog values for tank or arcade control
  // Master controller by default
  okapi::Controller controller;

  // Arm related objects
  okapi::ADIButton armLimitButton('H');
  okapi::ControllerButton armUpButton(E_CONTROLLER_DIGITAL_A);
  okapi::ControllerButton armDownButton(E_CONTROLLER_DIGITAL_B);
  okapi::Motor armMotor = 8_m;

  // Button to run our sample autonomous routine
  okapi::ControllerButton runAutoButton(E_CONTROLLER_DIGITAL_X);

  while (true) {
    // Tank drive with left and right sticks
    robotChassisController.tank(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                                controller.getAnalog(E_CONTROLLER_ANALOG_RIGHT_Y));

    // Arcade drive with the left stick
    robotChassisController.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                                  controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));

    // Don't power the arm if it is all the way down
    if (armLimitButton.isPressed()) {
      armMotor.move_voltage(0);
    } else {
      // Else, the arm isn't all the way down
      if (armUpButton.isPressed()) {
        armMotor.move_voltage(127);
      } else if (armDownButton.isPressed()) {
        armMotor.move_voltage(-127);
      } else {
        armMotor.move_voltage(0);
      }
    }

    // Run the test autonomous routine if we press the button
    if (runAutoButton.changedToPressed()) {
      // Drive the robot in a square pattern using closed-loop control
      for (int i = 0; i < 4; i++) {
        robotChassisController.moveDistance(2116); // Drive forward 12 inches
        robotChassisController.turnAngle(1662);    // Turn in place 90 degrees
      }
    }

    // Wait and give up the time we don't need to other tasks.
    // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
    pros::c::task_delay(10);
  }

  // FlywheelSimulator sim(0.01, 1, 0.5, 0.3, 0.05);
  // sim.setExternalTorqueFunction(
  //   [](double angle, double mass, double linkLen) { return (mass * -1 * gravity); });
  //
  // IterativePosPIDController controller(0.01, 0, 0);
  // controller.setTarget(radianToDegree * 1.5);
  // controller.setErrorScale(1);
  //
  // for (int i = 0; i < 1000000; i++) {
  //   sim.setTorque(controller.step(radianToDegree * sim.getAngle() * 20));
  //   sim.step();
  //   printf("%10.10f,%10.10f\n", sim.getAngle(), controller.getOutput());
  //   task_delay(10);
  // }
  //
  // return;

  // ChassisControllerIntegrated cci1(1_m, 2_m);
  // ChassisControllerIntegrated cci2(MotorGroup<2>({1_m, 2_rm}), MotorGroup<2>({3_m, 4_rm}));

  // ComposableFilter testFilter(
  //   {std::make_shared<MedianFilter<3>>(), std::make_shared<AverageFilter<5>>()});
  // VelMath velMath(1800, std::make_shared<ComposableFilter>(testFilter));

  // printf("velocity,current,efficiency,power,temperature,torque,voltage\n");
  // motor_move_velocity(9, 127);
  // while (true) {
  //   velMath.step(motor_get_position(9));
  //   printf("%1.2f,%lu,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f\n", velMath.getVelocity(),
  //          motor_get_current_draw(9), motor_get_efficiency(9), motor_get_power(9),
  //          motor_get_temperature(9), motor_get_torque(9), motor_get_voltage(9));
  //   task_delay(10);
  // }

  // OdomChassisControllerIntegrated occi1(1_m, 2_m, 1.0, 1.0);
  // OdomChassisControllerIntegrated occi2(
  //   std::make_shared<SkidSteerModel>(1_m, 2_m, ADIEncoder(1, 2), ADIEncoder(3, 4)), 1.0, 1.0);

  // OdomChassisControllerIntegrated helpMe(1_m, 2_m, 1.0, 1.0);

  // VelMath velMath(1800, 1, 0);
  // double mass = 0;
  // double accel = 0;
  // const double wheelDiam = 3.75;
  // double torque = 0;
  // double force = 0;
  // printf("request,velocity,filter\n");
  // MedianFilter<5> medFilt;
  // AverageFilter<5> filt;
  // double req = 0;
  // double increment = 1;
  // while (true) {
  //   torque = motor_get_torque(8);
  //
  //   const double pos = motor_get_position(8);
  //   velMath.step(pos);
  //   filt.filter(medFilt.filter(velMath.getVelocity()));
  //   accel = velMath.getAccel();
  //
  //   force = torque / (wheelDiam / 2.0);
  //   req += increment;
  //   if (req > 100) {
  //     req = 0;
  //     increment = 0;
  //   }
  //   motor_move_velocity(8, (int)req);
  //
  //   mass = force / accel;
  //   // printf("%1.2f,%1.2f\n", mass, accel);
  //   printf("%1.2f,%1.2f,\n", req, velMath.getVelocity());
  //   task_delay(10);
  // }

  {
    using namespace snowhouse;

    {
      test_printf("Testing ipow");

      test_printf("Integer tests");
      test("0^0 == 1", TEST_BODY(AssertThat, ipow(0, 0), Equals(1)));
      test("0^1 == 0", TEST_BODY(AssertThat, ipow(0, 1), Equals(0)));
      test("1^0 == 1", TEST_BODY(AssertThat, ipow(1, 0), Equals(1)));
      test("1^1 == 1", TEST_BODY(AssertThat, ipow(1, 1), Equals(1)));
      test("2^1 == 2", TEST_BODY(AssertThat, ipow(2, 1), Equals(2)));
      test("2^2 == 4", TEST_BODY(AssertThat, ipow(2, 2), Equals(4)));

      test_printf("Floating point tests");
      test("0.5^1 == 0.5", TEST_BODY(AssertThat, ipow(0.5, 1), EqualsWithDelta(0.5, 0.0001)));
      test("2.5^2 == 6.25", TEST_BODY(AssertThat, ipow(2.5, 2), EqualsWithDelta(6.25, 0.0001)));
    }

    {
      test_printf("Testing cutRange");

      test("1 : [-2, 2] -> 0",
           TEST_BODY(AssertThat, cutRange(1, -2, 2), EqualsWithDelta(0, 0.0001)));
      test("2 : [-2, 2] -> 0",
           TEST_BODY(AssertThat, cutRange(2, -2, 2), EqualsWithDelta(0, 0.0001)));
      test("-2 : [-2, 2] -> 0",
           TEST_BODY(AssertThat, cutRange(-2, -2, 2), EqualsWithDelta(0, 0.0001)));
      test("-3 : [-2, 2] -> -3",
           TEST_BODY(AssertThat, cutRange(-3, -2, 2), EqualsWithDelta(-3, 0.0001)));
      test("3 : [-2, 2] -> -3",
           TEST_BODY(AssertThat, cutRange(3, -2, 2), EqualsWithDelta(3, 0.0001)));
    }

    {
      test_printf("Testing remapRange");

      test("0 : [-1, 1] -> [-2, 2]",
           TEST_BODY(AssertThat, remapRange(0, -1, 1, -2, 2), EqualsWithDelta(0, 0.0001)));
      test("0.1 : [-1, 1] -> [-2, 2]",
           TEST_BODY(AssertThat, remapRange(0.1, -1, 1, -2, 2), EqualsWithDelta(0.2, 0.0001)));
      test("-0.1 : [-1, 1] -> [2, -2]",
           TEST_BODY(AssertThat, remapRange(-0.1, -1, 1, 2, -2), EqualsWithDelta(0.2, 0.0001)));
      test("0 : [-1, 1] -> [-5, 2]",
           TEST_BODY(AssertThat, remapRange(0, -1, 1, -5, 2), EqualsWithDelta(-1.5, 0.0001)));
    }

    {
      test_printf("Testing AverageFilter");

      AverageFilter<5> filt;

      for (int i = 0; i < 10; i++) {
        auto testName = "AverageFilter i = " + std::to_string(i);
        switch (i) {
        case 0: {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(0)));
          break;
        }

        case 1: {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.2, 0.01)));
          break;
        }

        case 2: {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.6, 0.01)));
          break;
        }

        case 3: {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(1.2, 0.01)));
          break;
        }

        default: {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(i - 2)));
          break;
        }
        }
      }
    }

    {
      test_printf("Testing MedianFilter");

      MedianFilter<5> filt;

      for (int i = 0; i < 10; i++) {
        auto testName = "MedianFilter i = " + std::to_string(i);
        if (i < 3) {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0, 0.0001)));
        } else {
          test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
        }
      }
    }

    {
      test_printf("Testing EmaFilter");

      EmaFilter filt(0.5);

      test("EmaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
      test("EmaFilter i = 1", TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.5, 0.0001)));
      test("EmaFilter i = 2", TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.25, 0.0001)));
      test("EmaFilter i = -3",
           TEST_BODY(AssertThat, filt.filter(-3), EqualsWithDelta(-0.875, 0.0001)));

      EmaFilter filt2(1);
      test("EmaFilter with alpha = 1 should return input signal 1",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
      test("EmaFilter with alpha = 1 should return input signal 2",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
      test("EmaFilter with alpha = 1 should return input signal 3",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    }

    {
      test_printf("Testing DemaFilter");

      DemaFilter filt(0.5, 0.05);

      test("DemaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
      test("DemaFilter i = 1",
           TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.525, 0.0001)));
      test("DemaFilter i = 2",
           TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.3244, 0.0001)));
      test("DemaFilter i = 2",
           TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.7410, 0.0001)));
      test("DemaFilter i = 2",
           TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.9557, 0.0001)));

      DemaFilter filt2(1, 0);
      test("DemaFilter with alpha = 1 and beta = 0 should return input signal 1",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
      test("DemaFilter with alpha = 1 and beta = 0 should return input signal 2",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
      test("DemaFilter with alpha = 1 and beta = 0 should return input signal 3",
           TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    }

    {
      test_printf("Testing EKFFilter");

      EKFFilter filt(0.0001, ipow(0.2, 2));

      test("EKFFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
      test("EKFFilter i = 0.5",
           TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.2454, 0.0001)));
      test("EKFFilter i = -0.5",
           TEST_BODY(AssertThat, filt.filter(-0.5), EqualsWithDelta(-0.0008, 0.0001)));
      test("EKFFilter i = 0.5",
           TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.1242, 0.0001)));
      test("EKFFilter i = 0",
           TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0.0992, 0.0001)));
    }

    {
      test_printf("Testing ComposableFilter");

      ComposableFilter filt(
        {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});

      test("ComposableFilter i = 1",
           TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.1111, 0.0001)));
      test("ComposableFilter i = 2",
           TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(0.4444, 0.0001)));
      test("ComposableFilter i = 3",
           TEST_BODY(AssertThat, filt.filter(3), EqualsWithDelta(1.1111, 0.0001)));

      for (int i = 4; i < 10; i++) {
        test("ComposableFilter i = " + std::to_string(i),
             TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
      }
    }

    {
      test_printf("Testing PassthroughFilter");

      PassthroughFilter filt;

      for (int i = 0; i < 5; i++) {
        test("PassthroughFilter i = " + std::to_string(i),
             TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i, 0.0001)));
      }
    }

    {
      test_printf("Testing Rate");

      Rate rate;
      uint32_t lastTime = pros::millis();

      for (int i = 0; i < 10; i++) {
        rate.delayHz(10);

        // Static cast so the compiler doesn't complain about comparing signed and unsigned values
        test("Rate " + std::to_string(i),
             TEST_BODY(AssertThat, static_cast<double>(pros::millis() - lastTime),
                       EqualsWithDelta(100, 10)));

        lastTime = pros::millis();
        task_delay(50); // Emulate some computation
      }
    }

    {
      test_printf("Testing VelMath");

      // DemaFilter gains 1 and 0 so it returns input signal and no filtering is performed
      VelMath velMath(360, std::make_shared<DemaFilter>(1.0, 0.0));

      for (int i = 0; i < 10; i++) {
        task_delay(100); // Delay first so the timestep works for the first iteration

        if (i == 0) {
          test("VelMath " + std::to_string(i),
               TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(0, 0.01)));
        } else {
          // 10 ticks per 100 ms should be ~16.67 rpm
          test("VelMath " + std::to_string(i),
               TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(16.67, 0.01)));
        }
      }
    }

    {
      test_printf("Testing FlywheelSimulator");

      // Default values
      FlywheelSimulator sim(0.01, 1, 0.5, 0.3, 0.005);
      sim.setExternalTorqueFunction([](double angle, double mass, double linkLen) {
        return (linkLen * std::cos(angle)) * (mass * -1 * gravity);
      });
      sim.setTorque(10);

      sim.step();

      test("FlywheelSimulator i = 0 angle",
           TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(0.0001237742, 0.00000001)));
      test("FlywheelSimulator i = 0 omega",
           TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(0.0247548337, 0.00000001)));
      test("FlywheelSimulator i = 0 accel",
           TEST_BODY(AssertThat, sim.getAcceleration(), EqualsWithDelta(990.19335, 0.0001)));
    }

    test_print_report();
  }

  while (true) {
    ADIButton btn(2);
    ControllerButton btn2(E_CONTROLLER_DIGITAL_A);
    btn.isPressed();
    btn.changed();
    btn.changedToPressed();
    btn.changedToReleased();

    ADIEncoder leftEncoder(1, 2, true);
    ADIEncoder rightEncoder(3, 4);
    leftEncoder.get();

    ADIUltrasonic ultra1(1, 2);
    ultra1.get();

    Motor mtr = 1_m;
    Motor r_mtr = 2_rm;

    ChassisControllerIntegrated int1(1_m,  // One motor on left side
                                     2_m); // One motor on right side

    ChassisControllerIntegrated int2(MotorGroup<3>({1_m, 2_m, 3_m}), // Three motors on left side
                                     MotorGroup<2>({4_m, 5_m}));     // Two motors on right side

    int1.moveDistance(0); // Closed-loop control
    int1.turnAngle(0);    // Closed-loop control

    int1.forward(0);                  // Open-loop control
    int1.rotate(0);                   // Open-loop control
    int1.driveVector(0, 0);           // Open-loop control
    int1.tank(0, 0);                  // Tank drive
    int1.arcade(0, 0);                // Arcade drive
    int1.left(0);                     // Left drive side
    int1.right(0);                    // Right drive side
    int1.stop();                      // Stop motors
    auto vals = int1.getSensorVals(); // Read left and right sensors
    int1.resetSensors();              // Set sensors to 0

    // A ChassisModel (SkidSteerModel or XDriveModel) is a simple interface to a robot chassis.
    // It is only an organization of motors and sensors, and is meant to be used by higher level
    // control systems that add closed-loop control, like ChassisController.
    SkidSteerModel model1(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}), leftEncoder,
                          rightEncoder);
    SkidSteerModel model2(MotorGroup<2>({1_m, 2_m}),
                          MotorGroup<2>({3_m, 4_m})); // Using integrated encoders
    XDriveModel xmodel1(1_m, 2_m, 3_m, 4_m, leftEncoder, rightEncoder);
    XDriveModel xmodel2(1_m, 2_m, 3_m, 4_m); // Using integrated encoders

    ChassisControllerPID controller1(
      std::make_shared<SkidSteerModel>(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}),
                                       leftEncoder, rightEncoder),
      IterativePosPIDControllerArgs(0, 0, 0), IterativePosPIDControllerArgs(0, 0, 0));

    ChassisControllerPID controller2(
      std::make_shared<XDriveModel>(1_m, 2_m, 3_m, 4_m, leftEncoder, rightEncoder),
      IterativePosPIDControllerArgs(0, 0, 0), IterativePosPIDControllerArgs(0, 0, 0));

    // An "odometry" chassis controller adds an odometry layer running in another task which keeps
    // track of the position of the robot in the odom frame. This means that you can tell the robot
    // to move to an arbitrary point on the field, or turn to an absolute angle (i.e., "turn to 90
    // degrees" will always put the robot facing east relative to the starting position)
    OdomChassisControllerPID controller3(
      std::make_shared<SkidSteerModel>(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}),
                                       leftEncoder, rightEncoder),
      0, 0, IterativePosPIDControllerArgs(0, 0, 0), IterativePosPIDControllerArgs(0, 0, 0));

    controller3.driveToPoint(0, 0); // Drive to (0, 0) on the field
    controller3.turnToAngle(0);     // Turn to 0 degrees

    IterativePosPIDController pid1(0, 0, 0); // PID controller
    IterativeVelPIDController tempvelpid1(0, 0);
    IterativeMotorController mc1(1_m, tempvelpid1);
    IterativeMotorController mc2(MotorGroup<2>({1_m, 2_m}), tempvelpid1);

    AsyncPosIntegratedController posI1(1_m);

    Motor tempMotor = 1_m;
    AsyncPosPIDController apospid1(leftEncoder, tempMotor, IterativePosPIDControllerArgs(0, 0, 0));
    AsyncPosPIDController apospid2(leftEncoder, tempMotor, 0, 0, 0);

    IterativePosPIDController pid2(0, 0, 0);
    IterativePosPIDController pid3(0, 0, 0, 0);
    IterativePosPIDController pid4(IterativePosPIDControllerArgs(0, 0, 0));
    IterativePosPIDController pid5(IterativePosPIDControllerArgs(0, 0, 0, 0));

    VelMath velMath1(0);
    VelMath velMath2(0, 0);
    VelMath velMath3(0, std::make_shared<DemaFilter>(0.0, 0.0));

    IterativeVelPIDController velPid1(0, 0);
    IterativeVelPIDController velPid2(IterativeVelPIDControllerArgs(0, 0));

    ADIEncoder quad1(0, 0);
    ADIEncoder quad2(0, 0, true);

    MotorGroup<2> mg1({1_m, 2_m});

    AverageFilter<2> avgFilt1;
    avgFilt1.filter(0);
    avgFilt1.getOutput();

    DemaFilter demaFilt1(0, 0);

    EKFFilter ekfFilter1;
    EKFFilter ekfFilter2(0);
    EKFFilter ekfFilter3(0, 0);

    EmaFilter emaFilt1(0);

    MedianFilter<5> medianFilt1;

    for (int i = 0; i < 10; i++) {
      printf("%d: %1.2f\n", i, avgFilt1.filter(i));
    }

    Odometry odom1(std::make_shared<SkidSteerModel>(MotorGroup<2>({1_m, 2_m}),
                                                    MotorGroup<2>({3_m, 4_m}), leftEncoder,
                                                    rightEncoder),
                   0, 0);

    Timer timer1();
  }
}
