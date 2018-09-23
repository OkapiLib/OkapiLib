#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  using namespace okapi;

  Logger::initialize(TimeUtilFactory::create().getTimer(), "/ser/sout", Logger::LogLevel::debug);

  auto drive =
    ChassisControllerFactory::create(-18, 19, AbstractMotor::gearset::green, {10.5_in, 4.125_in});

  auto mp = AsyncControllerFactory::motionProfile(1, 2, 10, drive);

  mp.generatePath({Point{0_ft, 0_ft, 0_deg}, Point{3_ft, 3_ft, 90_deg}, Point{3_ft, 4_ft, 45_deg}},
                  "E");
  mp.setTarget("E");
  mp.waitUntilSettled();

  //  auto rate = TimeUtilFactory::create().getRate();
  //
  //  std::uint32_t lastTime;
  //  std::uint32_t lastMillis = pros::millis();
  //  std::uint32_t nowMillis;
  //  const int delayAmount = 5;
  //
  //  while (true) {
  //    rate->delayUntil(delayAmount);
  //    if ((nowMillis = pros::millis()) - lastMillis != delayAmount) {
  //      printf("%d\n", (int)(nowMillis - lastMillis));
  //    }
  //    lastMillis = nowMillis;
  //  }
}
