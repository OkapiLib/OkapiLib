#ifndef OKAPI_MPCONSUMER
#define OKAPI_MPCONSUMER

#include <vector>
#include "motionProfile/motionProfile.h"
#include "control/velPid.h"

namespace okapi {
  class MPConsumerParams {
  public:
    MPConsumerParams(const float ikV, const float ikA, const float ikP = 0):
      kV(ikV),
      kA(ikA),
      kP(ikP) {}

    virtual ~MPConsumerParams() = default;

    float kV, kA, kP;
  };

  class MPConsumer {
  public:
    /**
     * Feed-forward controller for following a motion profile
     * @param kV      Velocity gain
     * @param kA      Acceleration gain
     * @param kP      Proportional gain
     */
    MPConsumer(const float ikV, const float ikA, const float ikP = 0):
      kV(ikV),
      kA(ikA),
      pid(ikP, 0),
      output(0) {}

    /**
     * Feed-forward controller for following a motion profile
     * @param iparams  mpConsumer params
     */
    MPConsumer(const MPConsumerParams& iparams):
      kV(iparams.kV),
      kA(iparams.kA),
      pid(iparams.kP, 0),
      output(0) {}

    virtual ~MPConsumer() = default;

    /**
     * Do one iteration of the controller
     * @param  profile    Motion profile to follow
     * @param  newReading New process measurement
     * @return            Controller output
     */
    virtual float step(const MotionProfile& profile, const float newReading);

    /**
     * Returns whether the motion profile has been completely followed
     * @return True if the motion profile is done
     */
    bool isComplete() const { return isCompleteFlag; }

    float getOutput() const { return output; }

    float getError() const { return pid.getError(); }

    void reset() {
      isCompleteFlag = false;
      pathStep = 0;
      pid.reset();
      output = 0;
    }
  private:
    bool isCompleteFlag = false;
    int pathStep = 0;
    const float kV, kA;
    VelPid pid;
    float output;
  };
}

#endif /* end of include guard: OKAPI_MPCONSUMER */
