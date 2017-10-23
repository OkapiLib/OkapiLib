#include "control/mpConsumer.h"

namespace okapi {
  float MPConsumer::step(const MotionProfile& profile, const float inewReading) {
    //Return the last output if we are done with the profile
    if (pathStep >= profile.size())
      return output;

    pid.setTarget(profile[pathStep].vel);
    output = pid.step(inewReading) + kV * profile[pathStep].vel + kA * profile[pathStep].accel;
    pathStep++;
    return output;
  }
}
