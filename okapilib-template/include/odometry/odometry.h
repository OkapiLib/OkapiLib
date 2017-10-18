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

  class Odometry {
  public:
    Odometry(const ChassisModelParams& imodelParams, const float iscale, const float iturnScale):
      model(imodelParams.make()),
      scale(iscale),
      turnScale(iturnScale),
      lastTicks{0, 0},
      mm(0) {}

    Odometry(const OdomParams& iparams):
      model(iparams.model),
      scale(iparams.scale),
      turnScale(iparams.turnScale),
      lastTicks{0, 0},
      mm(0) {}

    /**
     * Sets the parameters for Odometry math
     * @param iparams Odometry parameters
     */
    void setParams(OdomParams& iparams) {
      model = iparams.model;
      scale = iparams.scale;
      turnScale = iparams.turnScale;
    }

    /**
     * Set the drive and turn scales
     * @param iscale     Scale converting encoder ticks to mm
     * @param iturnScale Scale converting encoder ticks to radians
     */
    void setScales(const float iscale, const float iturnScale) {
      scale = iscale;
      turnScale = iturnScale;
    }

    /**
     * Attempt to guess scales based on robot dimensions
     * @param chassisDiam Center-to-center wheelbase diameter in inches
     * @param wheelDiam   Edge-to-edge wheel diameter in inches
     * @param ticksPerRev Quad ticks per revolution (default is 360)
     */
    void guessScales(const float chassisDiam, const float wheelDiam, const float ticksPerRev = 360.0);

    /**
     * Do one iteration of odometry math and return the new state estimate
     * @return               New state estimate
     */
    OdomState loop();

    static void trampoline(void *context) { static_cast<Odometry*>(context)->loop(); }

    OdomState getState() { return state; }
  private:
    std::shared_ptr<ChassisModel> model;
    OdomState state;
    float scale, turnScale;
    std::valarray<int> lastTicks;
    float mm;
  };
}

#endif /* end of include guard: OKAPI_ODOMETRY */
