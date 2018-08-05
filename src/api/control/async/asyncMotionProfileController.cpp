/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"

namespace okapi {
AsyncMotionProfileController::AsyncMotionProfileController(
  const TimeUtil &itimeUtil, const double imaxVel, const double imaxAccel, const double imaxJerk,
  std::shared_ptr<ChassisModel> imodel, QLength iwidth)
  : maxVel(imaxVel),
    maxAccel(imaxAccel),
    maxJerk(imaxJerk),
    model(imodel),
    width(iwidth),
    timeUtil(itimeUtil),
    logger(Logger::instance()),
    task(trampoline, this) {
}

AsyncMotionProfileController::~AsyncMotionProfileController() {
  dtorCalled = true;

  for (auto &path : paths) {
    free(path.second.left);
    free(path.second.right);
  }
}

void AsyncMotionProfileController::generatePath(std::initializer_list<Point> iwaypoints,
                                                std::string ipathId) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    logger->warn(
      "AsyncMotionProfileController: Not generating a path because no waypoints were given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      Waypoint{point.x.convert(meter), point.y.convert(meter), point.theta.convert(radian)});
  }

  TrajectoryCandidate candidate;
  logger->info("AsyncMotionProfileController: Preparing trajectory");
  pathfinder_prepare(points.data(), static_cast<int>(points.size()), FIT_HERMITE_CUBIC,
                     PATHFINDER_SAMPLES_FAST, 0.001, maxVel, maxAccel, maxJerk, &candidate);

  const int length = candidate.length;
  auto *trajectory = static_cast<Segment *>(malloc(length * sizeof(Segment)));

  logger->info("AsyncMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

  logger->info("AsyncMotionProfileController: Modifying for tank drive");
  pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, width.convert(meter));
  free(trajectory);

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    // Free the old path before overwriting it
    free(oldPath->second.left);
    free(oldPath->second.right);
    paths.erase(ipathId);
  }

  const auto lastWaypoint = points.back();
  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});
  logger->info("AsyncMotionProfileController: Completely done generating path");
}

void AsyncMotionProfileController::setTarget(std::string ipathId) {
  currentPath = ipathId;
  isRunning = true;
}

void AsyncMotionProfileController::loop() {
  auto rate = timeUtil.getRate();

  while (!dtorCalled) {
    if (!disabled && isRunning) {
      logger->info("AsyncMotionProfileController: Running with path: " + currentPath);
      auto path = paths.find(currentPath);

      if (path == paths.end()) {
        logger->warn(
          "AsyncMotionProfileController: Target was set to non-existent path with name: " +
          currentPath);
      } else {
        logger->debug("AsyncMotionProfileController: Path length is " +
                      std::to_string(path->second.length));

        executeSinglePath(path->second, timeUtil.getRate());
        model->stop();

        logger->info("AsyncMotionProfileController: Done moving");
      }

      isRunning = false;
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                     std::unique_ptr<AbstractRate> rate) {
  for (int i = 0; i < path.length && !disabled; ++i) {
    model->left(path.left[i].velocity / maxVel);
    model->right(path.right[i].velocity / maxVel);
    rate->delayUntil(1_ms);
  }
}

void AsyncMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncMotionProfileController *>(context)->loop();
  }
}

void AsyncMotionProfileController::waitUntilSettled() {
  logger->info("AsyncMotionProfileController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (isRunning) {
    rate->delayUntil(10_ms);
  }

  logger->info("AsyncMotionProfileController: Done waiting to settle");
}

Point AsyncMotionProfileController::getError() const {
  return Point{0_m, 0_m, 0_deg};
}

bool AsyncMotionProfileController::isSettled() {
  return !isRunning;
}

void AsyncMotionProfileController::reset() {
}

void AsyncMotionProfileController::flipDisable() {
  disabled = !disabled;
}

void AsyncMotionProfileController::flipDisable(bool iisDisabled) {
  disabled = iisDisabled;
}

bool AsyncMotionProfileController::isDisabled() const {
  return disabled;
}
} // namespace okapi
