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
      target(0),
      dt(15),
      profile(mpGen.generateProfile(dt)) {}

    MPController(const MPControllerParams& iparams):
      mpGen(iparams.mpGenParams),
      mpCon(iparams.mpConParams),
      target(0),
      dt(15),
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
    virtual float step(const float ireading) override { return mpCon.step(profile, ireading); }

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
      profile = mpGen.generateProfile(dt);
    }

    float getOutput() const override { return mpCon.getOutput(); }

    void reset() override { mpCon.reset(); }

    bool isComplete() const { return mpCon.isComplete(); }
  private:
    MPGenerator mpGen;
    MPConsumer mpCon;
    float target;
    int dt;
    MotionProfile profile;
  };
}

#endif /* end of include guard: OKAPI_MPCONTROLLER */
