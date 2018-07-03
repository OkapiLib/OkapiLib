#include "api.h"
#include "okapi/api.hpp"

void initialize() {
  auto timer = 5_s;
  okapi::PIDTuner tuner(std::make_shared<okapi::Motor>(1), std::make_shared<okapi::SettledUtil>(),
                        timer, 1000, 0.1, 2.0, 0.0001, 0.01, 20.0, 40.0);
  okapi::IterativePosPIDControllerArgs args = tuner.autotune();
  printf("%f %f %f\n", args.kP, args.kI, args.kD);
}

// the following functions don't work presently because comp. control
// hasn't been fully implemented
void disabled() {
}
void competition_initialize() {
}
