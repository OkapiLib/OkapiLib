#ifndef OKAPI_MPCONTROLLER
#define OKAPI_MPCONTROLLER

#include "control/mpConsumer.h"
#include "control/controlObject.h"
#include "motionProfile/mpGenerator.h"

namespace okapi {
  class MPControllerParams {
  public:
    MPControllerParams(const MPGenParams& igenParams, const MPConsumerParams& iconParams):
      mpGenParams(igenParams),
      mpConParams(iconParams) {}

    virtual ~MPControllerParams() = default;

    const MPGenParams& mpGenParams;
    const MPConsumerParams& mpConParams;
  };

  class MPController : public ControlObject {
  public:
    MPController(const MPGenParams& igenParams, const MPConsumerParams& iconParams):
      mpGen(igenParams),
      mpCon(iconParams),
      profile(mpGen.generateProfile(dt)) {}

    MPController(const MPControllerParams& iparams):
      mpGen(iparams.mpGenParams),
      mpCon(iparams.mpConParams),
      profile(mpGen.generateProfile(dt)) {}

    virtual ~MPController() {
      delete &mpGen;
      delete &mpCon;
    }

    /**
     * Do one iteration of the controller
     * @param  ireading New measurement
     * @return          Controller output
     */
    float step(const float ireading) override {
      if (isOn) {
        output = mpCon.step(profile, ireading);
        
        //Bound output
        if (output > max)
          output = max;
        else if (output < min)
         output = min;
      } else {
        output = 0; //Controller is off so write 0
      }

      return output;
    }

    /**
     * Set a new target position and regenerate the motion profile
     * @param itarget New target position
     */
    void setTarget(const float itarget) override {
      //Don't recalculate to get to the same spot
      if (itarget == target)
        return;

      target = itarget;
      mpGen.setTarget(itarget);
      profile = mpGen.generateProfile(dt); //Have to regen profile with new target
    }

    float getOutput() const override { return output; }

    float getError() const override { return mpCon.getError(); }

    void setSampleTime(const int isampleTime) override {
      dt = isampleTime;
      profile = mpGen.generateProfile(dt); //Have to regen profile with new dt
      reset();
    }

    void setOutputLimits(float imax, float imin) override { max = imax; min = imin; }

    void reset() override { mpCon.reset(); }

    void flipDisable() override { isOn = !isOn; }

    bool isComplete() const { return mpCon.isComplete(); }
  private:
    MPGenerator mpGen;
    MPConsumer mpCon;
    MotionProfile profile;
    float target = 0, output = 0, max = 127, min = -127;
    int dt = 0;
    bool isOn = true;
  };
}

#endif /* end of include guard: OKAPI_MPCONTROLLER */
