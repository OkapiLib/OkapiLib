#ifndef OKAPI_ODOMCHASSISCONTROLLER
#define OKAPI_ODOMCHASSISCONTROLLER

#include "odometry/odometry.h"
#include "chassis/basicChassisController.h"
#include "control/mpController.h"
#include <API.h>

namespace okapi {
  class OdomChassisController : public virtual ChassisController {
  public:
    /**
     * Odometry based chassis controller. Spins up a task at the default
     * priority plus 1 for odometry when constructed
     * @param iparams Odometry parameters for the internal odometry math
     */
    OdomChassisController(OdomParams iparams):
      ChassisController(iparams.model) {
        Odometry::setParams(iparams);
        taskCreate((TaskCode)&Odometry::loop, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT + 1);
      }

    virtual ~OdomChassisController() = default;

    /**
     * Drives the robot straight to a point in the odom frame
     * @param ix X coordinate
     * @param iy Y coordinate
     */
    virtual void driveToPoint(const float ix, const float iy, const bool ibackwards = false, const float ioffset = 0) = 0;

    /**
     * Turns the robot to face an angle in the odom frame
     * @param iangle Angle to turn to
     */
    virtual void turnToAngle(const float iangle) = 0;
  protected:
    static constexpr int moveThreshold = 100; //Minimum length movement
  };

  class OdomChassisControllerPid : public OdomChassisController, public ChassisControllerPid {
  public:
    OdomChassisControllerPid(OdomParams params, PidParams idistanceParams, PidParams iangleParams):
      ChassisController(params.model),
      OdomChassisController(params),
      ChassisControllerPid(params.model, idistanceParams, iangleParams) {}

    virtual ~OdomChassisControllerPid() = default;

    /**
     * Drives the robot straight to a point in the odom frame
     * @param ix X coordinate
     * @param iy Y coordinate
     */
    void driveToPoint(const float ix, const float iy, const bool ibackwards = false, const float ioffset = 0);

    /**
     * Turns the robot to face an angle in the odom frame
     * @param iangle Angle to turn to
     */
    void turnToAngle(const float iangle);
  };

  class OdomChassisControllerMP : public OdomChassisController, public ChassisControllerMP {
  public:
    OdomChassisControllerMP(OdomParams iparams, const MPControllerParams& iconParams):
      ChassisController(iparams.model),
      OdomChassisController(iparams),
      ChassisControllerMP(iparams.model, iconParams) {}

    virtual ~OdomChassisControllerMP() = default;

    /**
     * Drives the robot straight to a point in the odom frame
     * @param ix X coordinate
     * @param iy Y coordinate
     */
    void driveToPoint(const float ix, const float iy, const bool ibackwards = false, const float ioffset = 0);

    /**
     * Turns the robot to face an angle in the odom frame
     * @param iangle Angle to turn to
     */
    void turnToAngle(const float iangle);
  };
}

#endif /* end of include guard: OKAPI_ODOMCHASSISCONTROLLER */
