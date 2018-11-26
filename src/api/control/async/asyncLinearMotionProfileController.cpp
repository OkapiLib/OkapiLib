/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <numeric>

namespace okapi {
AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  const TimeUtil &itimeUtil,
  const PathfinderLimits &ilimits,
  const std::shared_ptr<ControllerOutput<double>> &ioutput,
  const QLength &idiameter,
  const AbstractMotor::GearsetRatioPair &ipair,
  const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    limits(ilimits),
    output(ioutput),
    diameter(idiameter),
    pair(ipair),
    timeUtil(itimeUtil) {
}

AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  AsyncLinearMotionProfileController &&other) noexcept
  : logger(other.logger),
    paths(std::move(other.paths)),
    limits(other.limits),
    output(std::move(other.output)),
    diameter(other.diameter),
    pair(other.pair),
    timeUtil(std::move(other.timeUtil)),
    currentPath(std::move(other.currentPath)),
    isRunning(other.isRunning.load(std::memory_order_acquire)),
    direction(other.direction.load(std::memory_order_acquire)),
    disabled(other.disabled.load(std::memory_order_acquire)),
    dtorCalled(other.dtorCalled.load(std::memory_order_acquire)),
    task(other.task) {
}

AsyncLinearMotionProfileController::~AsyncLinearMotionProfileController() {
  dtorCalled.store(true, std::memory_order_release);

  for (auto path : paths) {
    free(path.second.segment);
  }

  delete task;
}

void AsyncLinearMotionProfileController::generatePath(std::initializer_list<QLength> iwaypoints,
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
    points.push_back(Waypoint{point.convert(meter), 0, 0});
  }

  TrajectoryCandidate candidate;
  logger->info("AsyncLinearMotionProfileController: Preparing trajectory");
  pathfinder_prepare(points.data(),
                     static_cast<int>(points.size()),
                     FIT_HERMITE_CUBIC,
                     PATHFINDER_SAMPLES_FAST,
                     0.001,
                     limits.maxVel,
                     limits.maxAccel,
                     limits.maxJerk,
                     &candidate);

  const int length = candidate.length;

  if (length < 0) {
    auto pointToString = [](Waypoint point) {
      return "Point{x = " + std::to_string(point.x) + ", y = " + std::to_string(point.y) +
             ", theta = " + std::to_string(point.angle) + "}";
    };

    std::string message =
      "AsyncLinearMotionProfileController: The path (length " + std::to_string(length) +
      ") is impossible with waypoints: " +
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

  if (trajectory == nullptr) {
    std::string message =
      "AsyncLinearMotionProfileController: Could not allocate trajectory. The path (length " +
      std::to_string(length) + ") is probably impossible.";
    logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  logger->info("AsyncLinearMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  paths.emplace(ipathId, TrajectoryPair{trajectory, length});
  logger->info("AsyncLinearMotionProfileController: Completely done generating path");
  logger->info("AsyncLinearMotionProfileController: Path length: " + std::to_string(length));
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

void AsyncLinearMotionProfileController::setTarget(std::string ipathId) {
  setTarget(ipathId, false);
}

void AsyncLinearMotionProfileController::setTarget(std::string ipathId, const bool ibackwards) {
  currentPath = ipathId;
  isRunning.store(true, std::memory_order_release);
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
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

  while (!dtorCalled.load(std::memory_order_acquire)) {
    if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
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

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncLinearMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                           std::unique_ptr<AbstractRate> rate) {
  const auto reversed = direction.load(std::memory_order_acquire);

  for (int i = 0; i < path.length && !isDisabled(); ++i) {
    currentProfilePosition = path.segment[i].position;

    const auto motorRPM = convertLinearToRotational(path.segment[i].velocity * mps).convert(rpm);
    output->controllerSet(motorRPM / toUnderlyingType(pair.internalGearset) * reversed);

    rate->delayUntil(1_ms);
  }
}

QAngularSpeed AsyncLinearMotionProfileController::convertLinearToRotational(QSpeed linear) const {
  return (linear * (360_deg / (diameter * 1_pi))) * pair.ratio;
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

void AsyncLinearMotionProfileController::moveTo(const QLength &iposition,
                                                const QLength &itarget,
                                                const bool ibackwards) {
  std::string name = reinterpret_cast<const char *>(this); // hmmmm...
  generatePath({iposition, itarget}, name);
  setTarget(name, ibackwards);
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
  return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncLinearMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  auto rate = timeUtil.getRate();
  while (isRunning.load(std::memory_order_acquire)) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncLinearMotionProfileController::flipDisable() {
  flipDisable(!disabled.load(std::memory_order_acquire));
}

void AsyncLinearMotionProfileController::flipDisable(const bool iisDisabled) {
  logger->info("AsyncLinearMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  disabled.store(iisDisabled, std::memory_order_release);
  // loop() will set the output to 0 when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncLinearMotionProfileController::isDisabled() const {
  return disabled.load(std::memory_order_acquire);
}

void AsyncLinearMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this);
  }
}

void AsyncLinearMotionProfileController::tarePosition() {
}
} // namespace okapi
