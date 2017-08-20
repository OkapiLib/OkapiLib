#ifndef OKAPI_ODOMETRY
#define OKAPI_ODOMETRY

#include "chassis/chassisModel.h"
#include <valarray>
#include <memory>

namespace okapi {
  class OdomState {
  public:
    OdomState(const float ix, const float iy, const float itheta):
      x(ix),
      y(iy),
      theta(itheta) {}

    OdomState():
      x(0),
      y(0),
      theta(0) {}

    virtual ~OdomState() = default;

    float x, y, theta;
  };

  class OdomParams {
  public:
    OdomParams(const ChassisModelParams& iparams, const float iscale, const float iturnScale):
      model(iparams.make()),
      scale(iscale),
      turnScale(iturnScale) {}

    virtual ~OdomParams() = default;

    std::shared_ptr<ChassisModel> model;
    float scale, turnScale;
  };

  //Odometry has to be static because loop has to be called from a task
  class Odometry {
  public:
    /**
     * Sets the parameters for Odometry math
     * @param iparams Odometry parameters
     */
    static void setParams(OdomParams& iparams) {
      model = iparams.model;
      scale = iparams.scale;
      turnScale = iparams.turnScale;
    }

    /**
     * Set the drive and turn scales
     * @param iscale     Scale converting encoder ticks to mm
     * @param iturnScale Scale converting encoder ticks to deg
     */
    static void setScales(const float iscale, const float iturnScale) {
      scale = iscale;
      turnScale = iturnScale;
    }

    /**
     * Attempt to guess scales based on robot dimensions
     * @param chassisDiam Center-to-center wheelbase diameter in inches
     * @param wheelDiam   Edge-to-edge wheel diameter in inches
     * @param ticksPerRev Quad ticks per revolution (default is 360)
     */
    static void guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev = 360.0);

    /**
     * Do one iteration of odometry math and return the new state estimate
     * @return               New state estimate
     */
    static OdomState loop();

    static OdomState getState() { return state; }
  private:
    Odometry() {}
    Odometry(const Odometry& other);
    Odometry& operator=(Odometry& other);

    static std::shared_ptr<ChassisModel> model;
    static OdomState state;
    static float scale, turnScale;
    static std::valarray<int> lastTicks, currentTicks;
    static float mm;
  };
}

#endif /* end of include guard: OKAPI_ODOMETRY */
