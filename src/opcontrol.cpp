#include "api.h"
#include "okapi/api.hpp"

#include "test/testRunner.hpp"
#include "test/tests/impl/utilTests.hpp"

void runHeadlessTests() {
    using namespace okapi;

    runHeadlessUtilTests();

    test_print_report();
}

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
// using namespace okapi;
// auto drive = ChassisControllerFactory::create(-18,
//                                               19,
//                                               {},
//                                               {},
//                                               AbstractMotor::gearset::green,
//                                               {4.125_in, 10.5_in});
void opcontrol() {
    using namespace okapi;
    pros::Task::delay(100);

    //  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug);
    //  auto logger = Logger::instance();
    okapi::ADIEncoder leftEnc (7,8);
    okapi::ADIEncoder middleEnc (3,4);
    pros::ADIEncoder rightEnc (1,2);
    // auto drive = okapi::ChassisControllerFactory::createOdom(1,2,
    //                                                   leftEnc,
    //                                                   rightEnc,
    //                                                   middleEnc,
    //                                                   AbstractMotor::gearset::green,
    //                                                   {4_in, 18_in},
    //                                                   0_mm,
    //                                                   0_deg);
    pros::lcd::initialize();
    while (true) {
        // auto state = drive.getState();
        // pros::lcd::print(1, "x: %1.2f, y: %1.2f, theta: %1.2f",
        //        state.x.convert(inch),
        //        state.y.convert(inch),
        //        state.theta.convert(degree));
        pros::lcd::print(2, "%f %f %f", leftEnc.get(), middleEnc.get(), rightEnc.get_value());
        pros::delay(50);
    }


    pros::Task::delay(500);
}