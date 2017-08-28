#ifndef OKAPI_MOTIONPROFILE
#define OKAPI_MOTIONPROFILE

#include <vector>
#include "motionProfile/motionProfile.h"

/**
 * This motion profile code is heavily based off of Justin Marple's BNSLib code.
 * All credit goes to him for this code.
 * https://github.com/JMarple/BNSLibrary
 */

namespace okapi {
  /**
   * Motion profile generator parameters
   * @param maxAccel  Maximum acceleration
   * @param minAccel  Minimum acceleration
   * @param maxVel    Maximum velocity
   * @param targetPos Distance to travel
   * @param startVel  Starting velocity
   * @param endVel    Ending velocity
   */
  class MPGenParams {
  public:
    MPGenParams(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos, const float istartVel, const float iendVel):
      maxAccel(imaxAccel),
      minAccel(iminAccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(istartVel),
      endVel(iendVel) {}

    MPGenParams(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos):
      maxAccel(imaxAccel),
      minAccel(iminAccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(0),
      endVel(0) {}

    MPGenParams(const float iaccel, const float imaxVel, const float itargetPos):
      maxAccel(iaccel),
      minAccel(-1 * iaccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(0),
      endVel(0) {}

    MPGenParams(const float iaccel, const float imaxVel):
      maxAccel(iaccel),
      minAccel(-1 * iaccel),
      maxVel(imaxVel),
      targetPos(0),
      startVel(0),
      endVel(0) {}

    MPGenParams(const MPGenParams &other):
      maxAccel(other.maxAccel),
      minAccel(other.minAccel),
      maxVel(other.maxVel),
      targetPos(other.targetPos),
      startVel(other.startVel),
      endVel(other.endVel) {}

    virtual ~MPGenParams() = default;

    float maxAccel, minAccel, maxVel, targetPos, startVel, endVel;
  };

  class MPGenerator {
  public:
    /**
     * Motion profile generator
     * @param accel     Maximum acceleration
     * @param maxVel    Maximum velocity
     * @param targetPos Distance to travel
     */
    MPGenerator(const float iaccel, const float imaxVel):
      maxAccel(iaccel),
      minAccel(-1 * iaccel),
      maxVel(imaxVel),
      targetPos(0),
      startVel(0),
      endVel(0),
      isCompleteFlag(false),
      exchangeTime(determineExchangeTime(0)) {}

    /**
     * Motion profile generator
     * @param accel     Maximum acceleration
     * @param maxVel    Maximum velocity
     * @param targetPos Distance to travel
     */
    MPGenerator(const float iaccel, const float imaxVel, const float itargetPos):
      maxAccel(iaccel),
      minAccel(-1 * iaccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(0),
      endVel(0),
      isCompleteFlag(false),
      exchangeTime(determineExchangeTime(itargetPos)) {}

    /**
     * Motion profile generator
     * @param maxAccel  Maxiumum acceleration
     * @param minAccel  Minimum acceleration
     * @param maxVel    Maximum velocity
     * @param targetPos Distance to travel
     */
    MPGenerator(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos):
      maxAccel(imaxAccel),
      minAccel(iminAccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(0),
      endVel(0),
      isCompleteFlag(false),
      exchangeTime(determineExchangeTime(itargetPos)) {}


    /**
     * Motion profile generator
     * @param maxAccel  Maximum acceleration
     * @param minAccel  Minimum acceleration
     * @param maxVel    Maximum velocity
     * @param targetPos Distance to travel
     * @param startVel  Starting velocity
     * @param endVel    Ending velocity
     */
    MPGenerator(const float imaxAccel, const float iminAccel, const float imaxVel, const float itargetPos, const float istartVel, const float iendVel):
      maxAccel(imaxAccel),
      minAccel(iminAccel),
      maxVel(imaxVel),
      targetPos(itargetPos),
      startVel(istartVel),
      endVel(iendVel),
      isCompleteFlag(false),
      exchangeTime(determineExchangeTime(itargetPos)) {}

    /**
     * Motion profile generator
     * @param params Parameters (see MPGenParams docs)
     */
    MPGenerator(const MPGenParams& iparams):
      maxAccel(iparams.maxAccel),
      minAccel(iparams.minAccel),
      maxVel(iparams.maxVel),
      targetPos(iparams.targetPos),
      startVel(iparams.startVel),
      endVel(iparams.endVel),
      isCompleteFlag(false),
      exchangeTime(determineExchangeTime(iparams.targetPos)) {}

    virtual ~MPGenerator() = default;

    /**
     * Generates the total motion profile
     * @param dt Timestep
     * @return   Velocity targets in order of execution separated by dt
     */
    MotionProfile generateProfile(const float idt);

    /**
     * Gets the next target velocity in the motion profile
     * @param  time Current time in profile
     * @return      Next velocity target
     */
    MPTarget getNextVelTarget(const float itime);

    /**
     * Gets whether the profile is complete
     * @return True if complete
     */
    bool isComplete() const { return isCompleteFlag; }

    /**
     * Sets a new target position and regens the exchange time
     * @param itarget New target position
     */
    void setTarget(const float itarget) {
      targetPos = itarget;
      exchangeTime = determineExchangeTime(itarget);
    }
  protected:
    /**
     * Calculates the time at which, assuming no maximum velocity, the robot must start decelerating
     * @return Time of exchange
     */
    float determineExchangeTime(const float itarget) const;

    /**
     * Returns the next velocity target if the robot won't reach its max velocity
     * @param  time Time of target
     * @return      Next velocity target
     */
    MPTarget getVelWithoutMaxVel(const float time);

    /**
     * Returns the next velocity target if the robot will reach its max velocity
     * @param  time Time of target
     * @return      Next velocity target
     */
    MPTarget getVelWithMaxVel(const float time);
  private:
    const float maxAccel, minAccel, maxVel;
    float targetPos;
    const float startVel, endVel;
    bool isCompleteFlag;
    float exchangeTime;
  };
}

#endif /* end of include guard: OKAPI_MOTIONPROFILE */
