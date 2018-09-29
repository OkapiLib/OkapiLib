/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include <numeric>

namespace okapi {
AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  const TimeUtil &itimeUtil,
  const double imaxVel,
  const double imaxAccel,
  const double imaxJerk,
  std::shared_ptr<ControllerOutput<double>> ioutput)
  : logger(Logger::instance()),
    maxVel(imaxVel),
    maxAccel(imaxAccel),
    maxJerk(imaxJerk),
    output(ioutput),
    timeUtil(itimeUtil) {
}

AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  AsyncLinearMotionProfileController &&other) noexcept
  : logger(other.logger),
    paths(std::move(other.paths)),
    maxVel(other.maxVel),
    maxAccel(other.maxAccel),
    maxJerk(other.maxJerk),
    output(std::move(other.output)),
    timeUtil(std::move(other.timeUtil)),
    currentPath(std::move(other.currentPath)),
    isRunning(other.isRunning),
    disabled(other.disabled),
    dtorCalled(other.dtorCalled.load(std::memory_order::memory_order_relaxed)),
    task(other.task) {
}

AsyncLinearMotionProfileController::~AsyncLinearMotionProfileController() {
  dtorCalled.store(true, std::memory_order::memory_order_relaxed);

  for (auto path : paths) {
    free(path.second.segment);
  }

  delete task;
}

void AsyncLinearMotionProfileController::generatePath(std::initializer_list<double> iwaypoints,
                                                      const std::string &ipathId) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    logger->warn(
      "AsyncLinearMotionProfileController: Not generating a path because no waypoints were given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(Waypoint{point, 0, 0});
  }

  TrajectoryCandidate candidate;
  logger->info("AsyncLinearMotionProfileController: Preparing trajectory");
  pathfinder_prepare(points.data(),
                     static_cast<int>(points.size()),
                     FIT_HERMITE_CUBIC,
                     PATHFINDER_SAMPLES_FAST,
                     0.001,
                     maxVel,
                     maxAccel,
                     maxJerk,
                     &candidate);

  const int length = candidate.length;

  if (length < 0) {
    auto pointToString = [](Waypoint point) {
      return "Point{x = " + std::to_string(point.x) + ", y = " + std::to_string(point.y) +
             ", theta = " + std::to_string(point.angle) + "}";
    };

    std::string message =
      "AsyncLinearMotionProfileController: Path is impossible with waypoints: " +
      std::accumulate(std::next(points.begin()),
                      points.end(),
                      pointToString(points.at(0)),
                      [&](std::string a, Waypoint b) { return a + ", " + pointToString(b); });

    logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  auto *trajectory = static_cast<Segment *>(malloc(length * sizeof(Segment)));

  logger->info("AsyncLinearMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  paths.emplace(ipathId, TrajectoryPair{trajectory, length});
  logger->info("AsyncLinearMotionProfileController: Completely done generating path");
  logger->info("AsyncLinearMotionProfileController: " + std::to_string(length));
}

void AsyncLinearMotionProfileController::removePath(const std::string &ipathId) {
  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    free(oldPath->second.segment);
    paths.erase(ipathId);
  }
}

std::vector<std::string> AsyncLinearMotionProfileController::getPaths() {
  std::vector<std::string> keys;

  for (const auto &path : paths) {
    keys.push_back(path.first);
  }

  return keys;
}

void AsyncLinearMotionProfileController::setTarget(const std::string ipathId) {
  currentPath = ipathId;
  isRunning = true;
}

void AsyncLinearMotionProfileController::controllerSet(const std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncLinearMotionProfileController::getTarget() {
  return currentPath;
}

std::string AsyncLinearMotionProfileController::getTarget() const {
  return currentPath;
}

void AsyncLinearMotionProfileController::loop() {
  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order::memory_order_relaxed)) {
    if (isRunning && !isDisabled()) {
      logger->info("AsyncLinearMotionProfileController: Running with path: " + currentPath);
      auto path = paths.find(currentPath);

      if (path == paths.end()) {
        logger->warn(
          "AsyncLinearMotionProfileController: Target was set to non-existent path with name: " +
          currentPath);
      } else {
        logger->debug("AsyncLinearMotionProfileController: Path length is " +
                      std::to_string(path->second.length));

        executeSinglePath(path->second, timeUtil.getRate());
        output->controllerSet(0);

        logger->info("AsyncLinearMotionProfileController: Done moving");
      }

      isRunning = false;
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncLinearMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                           std::unique_ptr<AbstractRate> rate) {
  for (int i = 0; i < path.length && !isDisabled(); ++i) {
    currentProfilePosition = path.segment[i].position;
    output->controllerSet(path.segment[i].velocity / maxVel);
    rate->delayUntil(1_ms);
  }
}

void AsyncLinearMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncLinearMotionProfileController *>(context)->loop();
  }
}

void AsyncLinearMotionProfileController::waitUntilSettled() {
  logger->info("AsyncLinearMotionProfileController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  logger->info("AsyncLinearMotionProfileController: Done waiting to settle");
}

void AsyncLinearMotionProfileController::moveTo(double iposition, double itarget) {
  std::string name = reinterpret_cast<const char *>(this); // hmmmm...
  generatePath({iposition, itarget}, name);
  setTarget(name);
  waitUntilSettled();
  removePath(name);
}

double AsyncLinearMotionProfileController::getError() const {
  if (const auto path = paths.find(getTarget()); path == paths.end()) {
    return 0;
  } else {
    // The last position in the path is the target position
    return path->second.segment[path->second.length - 1].position - currentProfilePosition;
  }
}

bool AsyncLinearMotionProfileController::isSettled() {
  return isDisabled() || !isRunning;
}

void AsyncLinearMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  auto rate = timeUtil.getRate();
  while (isRunning) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncLinearMotionProfileController::flipDisable() {
  flipDisable(!disabled);
}

void AsyncLinearMotionProfileController::flipDisable(const bool iisDisabled) {
  logger->info("AsyncLinearMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  disabled = iisDisabled;
  // loop() will set the output to 0 when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncLinearMotionProfileController::isDisabled() const {
  return disabled;
}

void AsyncLinearMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this);
  }
}
} // namespace okapi
