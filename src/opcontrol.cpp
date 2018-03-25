#include "api.h"

#include "okapi/api.hpp"
#include "okapi/test/testRunner.hpp"

using namespace okapi;

void opcontrol() {
  task_delay(100);

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

    test_print_report();
  }

  /*while (true) {
    ADIButton btn(2);
    ControllerButton btn2(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A);
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
      SkidSteerModelParams(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}), leftEncoder,
                           rightEncoder),
      PosPIDControllerParams(0, 0, 0), PosPIDControllerParams(0, 0, 0));

    ChassisControllerPID controller2(
      XDriveModelParams(1_m, 2_m, 3_m, 4_m, leftEncoder, rightEncoder),
      PosPIDControllerParams(0, 0, 0), PosPIDControllerParams(0, 0, 0));

    // An "odometry" chassis controller adds an odometry layer running in another task which keeps
    // track of the position of the robot in the odom frame. This means that you can tell the robot
    // to move to an arbitrary point on the field, or turn to an absolute angle (i.e., "turn to 90
    // degrees" will always put the robot facing east relative to the starting position)
    OdomChassisControllerPID controller3(
      OdometryParams(SkidSteerModelParams(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}),
                                          leftEncoder, rightEncoder),
                     0, 0),
      PosPIDControllerParams(0, 0, 0), PosPIDControllerParams(0, 0, 0));

    OdomChassisControllerPID controller4(
      OdometryParams(XDriveModelParams(1_m, 2_m, 3_m, 4_m, leftEncoder, rightEncoder), 0, 0),
      PosPIDControllerParams(0, 0, 0), PosPIDControllerParams(0, 0, 0));

    controller3.driveToPoint(0, 0); // Drive to (0, 0) on the field
    controller3.turnToAngle(0);     // Turn to 0 degrees

    PosPIDController pid1(0, 0, 0); // PID controller
    MotorController mc1(1_m, pid1); // Motor controller with one motor and the PID controller
    MotorController mc2(MotorGroup<2>({1_m, 2_m}),
                        pid1); // Motor controller with two motors and the PID controller

    PosIntegratedController posI1(1_m);

    Motor tempMotor = 1_m;
    AsyncPosPIDController apospid1(leftEncoder, tempMotor, PosPIDControllerParams(0, 0, 0));
    AsyncPosPIDController apospid2(leftEncoder, tempMotor, 0, 0, 0);

    PosPIDController pid2(0, 0, 0);
    PosPIDController pid3(0, 0, 0, 0);
    PosPIDController pid4(PosPIDControllerParams(0, 0, 0));
    PosPIDController pid5(PosPIDControllerParams(0, 0, 0, 0));

    VelMath velMath1(0);
    VelMath velMath2(0, 0);
    VelMath velMath3(0, 0, 0);

    VelPIDController velPid1(0, 0);
    VelPIDController velPid2(VelPIDControllerParams(0, 0));

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

    Odometry odom1(SkidSteerModelParams(MotorGroup<2>({1_m, 2_m}), MotorGroup<2>({3_m, 4_m}),
                                        leftEncoder, rightEncoder),
                   0, 0);

    Timer timer1();
  }*/
}
