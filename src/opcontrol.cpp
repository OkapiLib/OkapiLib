#include "api.h"

#include "okapi/api.hpp"
#include "test/testRunner.hpp"
#include "test/tests/impl/odometryTests.hpp"
#include "test/tests/impl/utilTests.hpp"

void runHeadlessTests();

void constructorTests();

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  runHeadlessUtilTests();
  runHeadlessOdometryTests();
  return;

  MotorGroup leftMotors({19_mtr, 20_mtr});
  MotorGroup rightMotors({13_rmtr, 14_rmtr});
  Motor armMotor = 15_mtr;
  armMotor.move(10);

  auto chassis =
    ChassisControllerFactory::create({19, 20}, {-14}, AbstractMotor::gearset::red, {4_in, 11.5_in});

  Controller controller;
  ControllerButton btn1(E_CONTROLLER_DIGITAL_A);
  ControllerButton btn2(E_CONTROLLER_DIGITAL_B);
  ControllerButton btn3(E_CONTROLLER_DIGITAL_Y);
  ControllerButton btn4(E_CONTROLLER_DIGITAL_X);

  while (true) {
    chassis.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                   controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));

    if (btn1.changedToPressed()) {
      printf("move distance\n");
      chassis.moveDistance(12_in);
    }

    if (btn2.changedToPressed()) {
      printf("turn angle\n");
      chassis.turnAngle(90_deg);
    }

    if (btn3.changedToPressed()) {
      printf("move arm\n");
      armMotor.moveRelative(-10, 127);
    }

    if (btn4.changedToPressed()) {
      printf("autonomous routine\n");
      for (int i = 0; i < 4; i++) {
        chassis.moveDistance(12_in);
        chassis.turnAngle(90_deg);
      }
    }

    pros::Task::delay(10);
  }
}

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessUtilTests();

  test_print_report();
}

void constructorTests() {
  using namespace okapi;

  {
    Controller help;
    ControllerButton btn = help[E_CONTROLLER_DIGITAL_A];
  }

  {
    ADIButton btn(2);
    ControllerButton btn2(E_CONTROLLER_DIGITAL_A);
    btn.isPressed();
    btn.changed();
    btn.changedToPressed();
    btn.changedToReleased();
  }

  {
    ADIEncoder leftEncoder(1, 2, true);
    ADIEncoder rightEncoder(3, 4);
    ADIEncoder test('A', 'B');
    ADIEncoder test2('a', 'b');
    leftEncoder.get();
  }

  {
    Potentiometer pot1(1);
    Potentiometer pot2('a');
    Potentiometer pot3('A');
    pot1.get();
  }

  {
    ADIUltrasonic ultra1(1, 2);
    ultra1.get();
  }

  {
    Motor mtr = 1_mtr;
    Motor r_mtr = 2_rmtr;
  }

  {
    ADIEncoder leftEncoder(1, 2, true);
    ADIEncoder rightEncoder(3, 4);
    ChassisControllerIntegrated int1 = ChassisControllerFactory::create(1_mtr, 2_mtr);
    ChassisControllerIntegrated int2 = ChassisControllerFactory::create(
      MotorGroup({1_mtr, 2_mtr, 3_mtr}), MotorGroup({4_mtr, 5_mtr}));
    ChassisControllerIntegrated int3 = ChassisControllerFactory::create(1, 2);
    ChassisControllerIntegrated int4 = ChassisControllerFactory::create({1, 2, 3}, {-4, -5});
    ChassisControllerIntegrated int5 =
      ChassisControllerFactory::create(1, 2, AbstractMotor::gearset::red, {1, 1});
    ChassisControllerIntegrated int6 =
      ChassisControllerFactory::create({1, 2}, {3, 4}, AbstractMotor::gearset::red, {1, 1});
    ChassisControllerIntegrated int7 =
      ChassisControllerFactory::create({1, 2}, {3, 4}, AbstractMotor::gearset::red * 2, {1, 1});
    ChassisControllerIntegrated int8 = ChassisControllerFactory::create(
      {1, 2}, {3, 4}, AbstractMotor::gearset::red * (2 / 3), {1, 1});

    int1.moveDistance(0_in); // Closed-loop control
    int1.turnAngle(0_deg);   // Closed-loop control

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
  }

  {
    VelMath velMath1(0, std::make_shared<DemaFilter>(0.0, 0.0), std::make_unique<Timer>());
    VelMath velMath2 = VelMathFactory::create(0);
    VelMath velMath3 = VelMathFactory::create(0, std::make_shared<EmaFilter>(0.0));
  }

  {
    ADIEncoder quad1(0, 0);
    ADIEncoder quad2(0, 0, true);
  }

  {
    MotorGroup mg1({1_mtr, 2_mtr});
    MotorGroup mg2({1, -2});
    MotorGroup mg3({Motor(1), Motor(2)});
  }

  {
    AverageFilter<2> avgFilt1;
    avgFilt1.filter(0);
    avgFilt1.getOutput();
  }

  { DemaFilter demaFilt1(0, 0); }

  {
    EKFFilter ekfFilter1;
    EKFFilter ekfFilter2(0);
    EKFFilter ekfFilter3(0, 0);
  }

  { EmaFilter emaFilt1(0); }

  { MedianFilter<5> medianFilt1; }

  { Timer timer1(); }

  {
    auto controllerRunner = ControllerRunnerFactory::create();
    AsyncPosIntegratedController testControllerRunnerController1(
      std::make_shared<Motor>(1), std::make_unique<SettledUtil>(std::make_unique<Timer>()));
  }

  {
    SettledUtil settledUtil1(std::make_unique<Timer>());
    settledUtil1.isSettled(0);
  }

  {
    auto mtr = 1_mtr;
    AsyncVelPIDController con(std::make_shared<IntegratedEncoder>(mtr),
                              std::make_shared<Motor>(mtr), std::make_unique<Rate>(),
                              std::make_unique<Timer>(), SettledUtilFactory::createPtr(), 0, 0, 0,
                              VelMathFactory::createPtr(imev5TPR));
  }

  {
    auto mtr = 1_mtr;
    AsyncWrapper wrapper(std::make_shared<IntegratedEncoder>(mtr), std::make_shared<Motor>(mtr),
                         std::make_unique<IterativePosPIDController>(
                           0, 0, 0, 0, std::make_unique<Timer>(), SettledUtilFactory::createPtr()),
                         std::make_unique<Rate>());
  }
}
