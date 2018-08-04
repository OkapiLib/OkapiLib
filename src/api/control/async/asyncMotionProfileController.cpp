/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"

namespace okapi {
AsyncMotionProfileController::AsyncMotionProfileController(const TimeUtil &itimeUtil,
                                                           double imaxVel, double imaxAccel,
                                                           double imaxJerk,
                                                           std::shared_ptr<SkidSteerModel> imodel,
                                                           QLength iwidth)
  : maxVel(imaxVel),
    maxAccel(imaxAccel),
    maxJerk(imaxJerk),
    model(imodel),
    width(iwidth),
    rate(std::move(itimeUtil.getRate())),
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

  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});
  logger->info("AsyncMotionProfileController: Completely done generating path");
}

void AsyncMotionProfileController::executePath(std::string ipathId) {
  currentPath = ipathId;
  isRunning = true;
}

void AsyncMotionProfileController::loop() {
  while (!dtorCalled) {
    if (isRunning) {
      logger->info("AsyncMotionProfileController: Running with path: " + currentPath);
      const auto path = paths.at(currentPath);

      for (int i = 0; i < path.length; ++i) {
        model->left(path.left[i].velocity / maxVel);
        model->right(path.right[i].velocity / maxVel);
        rate->delayUntil(1_ms);
      }

      isRunning = false;
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncMotionProfileController *>(context)->loop();
  }
}
} // namespace okapi
