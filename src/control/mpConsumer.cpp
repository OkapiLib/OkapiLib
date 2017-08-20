#include "control/mpConsumer.h"

namespace okapi {
  float MPConsumer::loop(const MotionProfile& profile, const float inewReading) {
    //Return the last output if we are done with the profile
    if (step >= profile.size())
      return output;

    pid.setTarget(profile[step].vel);
    output = pid.loop(inewReading) + kV * profile[step].vel + kA * profile[step].accel;
    step++;
    return output;
  }
}
