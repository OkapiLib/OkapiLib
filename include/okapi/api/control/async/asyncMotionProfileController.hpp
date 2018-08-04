/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCMOTIONPROFILECONTROLLER_HPP_
#define _OKAPI_ASYNCMOTIONPROFILECONTROLLER_HPP_

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/control/async/asyncPositionController.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <map>

extern "C" {
#include "pathfinder.h"
}

namespace okapi {
struct Point {
  QLength x;    // X coordinate relative to the start of the movement
  QLength y;    // Y coordinate relative to the start of the movement
  QAngle theta; // Exit angle relative to the start of the movement

  Point operator+(const Point &other) const {
    return Point{x + other.x, y + other.y, theta + other.theta};
  }

  Point operator-(const Point &other) const {
    return Point{x - other.x, y - other.y, theta - other.theta};
  }
};

class AsyncMotionProfileController : public AsyncPositionController<std::string, Point> {
  public:
  /**
   * An Async Controller which generates and follows 2D motion profiles.
   *
   * @param imaxVel The maximum possible velocity.
   * @param imaxAccel The maximum possible acceleration.
   * @param imaxJerk The maximum possible jerk.
   * @param imodel The chassis model to control.
   * @param iwidth The chassis wheelbase width.
   */
  AsyncMotionProfileController(const TimeUtil &itimeUtil, double imaxVel, double imaxAccel,
                               double imaxJerk, std::shared_ptr<SkidSteerModel> imodel,
                               QLength iwidth);

  ~AsyncMotionProfileController() override;

  /**
   * Generates a path which intersects the given waypoints and saves it internally with a key of
   * pathId. Call executePath() with the same pathId to run it.
   *
   * @param iwaypoints The Waypoints to hit on the path.
   * @param ipathId A unique identifier to save the path with.
   */
  void generatePath(std::initializer_list<Point> iwaypoints, std::string ipathId);

  /**
   * Executes a path with the given ID. If there is no path matching the ID, the method will
   * return. Any targets set while a path is being followed will be ignored.
   *
   * @param ipathId A unique identifier for the path, previously passed to generatePath().
   */
  void setTarget(std::string ipathId) override;

  void waitUntilSettled() override;

  Point getError() const override;

  bool isSettled() override;

  void reset() override;

  void flipDisable() override;

  void flipDisable(bool iisDisabled) override;

  bool isDisabled() const override;

  protected:
  struct TrajectoryPair {
    Segment *left;
    Segment *right;
    int length;
    Point finalPosition;
  };

  std::map<std::string, TrajectoryPair> paths{};
  double maxVel{0};
  double maxAccel{0};
  double maxJerk{0};
  std::shared_ptr<SkidSteerModel> model;
  QLength width{11_in};
  TimeUtil timeUtil;
  Logger *logger;

  CrossplatformThread task;
  bool dtorCalled{false};
  std::string currentPath{""};
  Point currentPosition{0_m, 0_m, 0_deg};
  bool isRunning{false};
  bool disabled{false};

  static void trampoline(void *context);
  void loop();
};
} // namespace okapi

#endif
