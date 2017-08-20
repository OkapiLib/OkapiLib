#ifndef OKAPI_MPCONTROLLER
#define OKAPI_MPCONTROLLER

#include "control/mpConsumer.h"
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

  class MPController {
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
     * @param  inewReading New measurement
     * @return            Controller output
     */
    virtual float loop(const float inewReading) { return mpCon.loop(profile, inewReading); }

    /**
     * Set a new target position and regenerate the motion profile
     * @param pos New target position
     */
    void setTarget(const int pos) {
      //Don't recalculate to get to the same spot
      if (pos == target)
        return;

      target = pos;
      mpGen.setTarget(pos);
      profile = mpGen.generateProfile(dt);
    }

    bool isComplete() const { return mpCon.isComplete(); }

    void reset() { mpCon.reset(); }
  private:
    MPGenerator mpGen;
    MPConsumer mpCon;
    int target, dt;
    MotionProfile profile;
  };
}

#endif /* end of include guard: OKAPI_MPCONTROLLER */
