#include "api.h"
#include "okapi/api.hpp"

#include "test/testRunner.hpp"
#include "test/tests/impl/utilTests.hpp"

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessUtilTests();

  test_print_report();
}

std::atomic_bool baz{false};

void foo(void *) {
  while (true) {
    bool val = baz.load(std::memory_order::memory_order_relaxed);
    if (val) {
      printf("foo\n");
    }
    baz.store(!val, std::memory_order::memory_order_relaxed);
  }
}

void barr(void *) {
  while (true) {
    bool val = baz.load(std::memory_order::memory_order_relaxed);
    if (val) {
      printf("bar\n");
    }
    baz.store(!val, std::memory_order::memory_order_relaxed);
  }
}

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  //  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug);
  //  auto logger = Logger::instance();

  pros::Task fooTask(foo);
  pros::Task barTask(barr);
  pros::Task::delay(1000);

  auto drive = ChassisControllerFactory::createOdom(
    -1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in}, 0_mm);
  drive.driveToPoint(4_in, 0_in);

  auto state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(4_in, 4_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(0_in, 4_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(0_in, 0_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  pros::Task::delay(500);

  //  {
  //    auto model =
  //      std::make_shared<SkidSteerModel>(std::make_shared<Motor>(-1), std::make_shared<Motor>(2));
  //    auto cnt = AsyncControllerFactory::motionProfile(1.0, 2.0, 10.0, model, 10.5_in);
  //
  //    cnt.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3_ft, 0_ft, 0_deg}}, "A");
  //    cnt.setTarget("A");
  //    cnt.waitUntilSettled();
  //  }

  //  auto model =
  //    std::make_shared<SkidSteerModel>(std::make_shared<Motor>(-1), std::make_shared<Motor>(2));
  //  auto cnt = AsyncControllerFactory::motionProfile(1.0, 2.0, 10.0, model, 10.5_in);
  //
  //  cnt.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3_ft, 0_ft, 0_deg}}, "A");
  //  cnt.setTarget("B");
  //  cnt.waitUntilSettled();

  //  auto drive =
  //    ChassisControllerFactory::create(-1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in});
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();
  //  auto drive = ChassisControllerFactory::create(
  //    -1,
  //    2,
  //                                                IterativePosPIDController::Gains{0.01, 0, 0,
  //                                                0}, IterativePosPIDController::Gains{0, 0, 0,
  //                                                0}, IterativePosPIDController::Gains{0.007, 0,
  //                                                0, 0},
  //    AbstractMotor::gearset::red,
  //    {2.5_in, 10.5_in});
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();

  //  auto cnt = AsyncControllerFactory::posPID(2, 0.01, 0, 0);
  //  cnt.setTarget(360);
  //  cnt.waitUntilSettled();
  //  logger->debug("opcontrol: position: " + std::to_string(Motor(2).getPosition()));

  //  runHeadlessTests();
  return;

  MotorGroup leftMotors({19_mtr, 20_mtr});
  MotorGroup rightMotors({13_rmtr, 14_rmtr});
  Motor armMotor = 15_mtr;
  armMotor.move(10);

  auto chassis =
    ChassisControllerFactory::create({19, 20}, {-14}, AbstractMotor::gearset::red, {4_in, 11.5_in});

  Controller controller;
  ControllerButton btn1(ControllerDigital::A);
  ControllerButton btn2(ControllerDigital::B);
  ControllerButton btn3(ControllerDigital::Y);
  ControllerButton btn4(ControllerDigital::X);

  while (true) {
    pros::lcd::print(0,
                     "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
    int left = master.get_analog(ANALOG_LEFT_Y);
    int right = master.get_analog(ANALOG_RIGHT_Y);

    left_mtr = left;
    right_mtr = right;
    pros::delay(20);
  }
}
