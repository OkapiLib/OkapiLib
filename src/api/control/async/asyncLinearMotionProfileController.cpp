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

AsyncLinearMotionProfileController::~AsyncLinearMotionProfileController() {
  dtorCalled.store(true, std::memory_order_release);

  for (auto path : paths) {
    free(path.second.segment);
  }

  delete task;
}

void AsyncLinearMotionProfileController::generatePath(std::initializer_list<QLength> iwaypoints,
                                                      const std::string &ipathId) {
  generatePath(iwaypoints, ipathId, limits);
}

void AsyncLinearMotionProfileController::generatePath(std::initializer_list<QLength> iwaypoints,
                                                      const std::string &ipathId,
                                                      const PathfinderLimits &ilimits) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    LOG_WARN_S("AsyncLinearMotionProfileController: Not generating a path because no waypoints were"
               " given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(Waypoint{point.convert(meter), 0, 0});
  }

  LOG_INFO_S("AsyncLinearMotionProfileController: Preparing trajectory");

  TrajectoryCandidate candidate;
  pathfinder_prepare(points.data(),
                     static_cast<int>(points.size()),
                     FIT_HERMITE_CUBIC,
                     PATHFINDER_SAMPLES_FAST,
                     0.010,
                     ilimits.maxVel,
                     ilimits.maxAccel,
                     ilimits.maxJerk,
                     &candidate);

  const int length = candidate.length;

  if (length < 0) {
    std::string message = "AsyncLinearMotionProfileController: Length was negative. " +
                          getPathErrorMessage(points, ipathId, length);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  auto *trajectory = static_cast<Segment *>(malloc(length * sizeof(Segment)));

  if (trajectory == nullptr) {
    std::string message = "AsyncLinearMotionProfileController: Could not allocate trajectory. " +
                          getPathErrorMessage(points, ipathId, length);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  LOG_INFO_S("AsyncLinearMotionProfileController: Generating path");

  pathfinder_generate(&candidate, trajectory);

  // Free the old path before overwriting it
  if (!removePath(ipathId)) {
    std::string message = "AsyncLinearMotionProfileController: Could not remove path " + ipathId + 
                          " to overwrite. Path may be running.";
    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  paths.emplace(ipathId, TrajectoryPair{trajectory, length});

  LOG_INFO("AsyncLinearMotionProfileController: Completely done generating path " + ipathId);
  LOG_DEBUG("AsyncLinearMotionProfileController: Path length: " + std::to_string(length));
}

std::string
AsyncLinearMotionProfileController::getPathErrorMessage(const std::vector<Waypoint> &points,
                                                        const std::string &ipathId,
                                                        const int length) {
  auto pointToString = [](Waypoint point) {
    return "Point{x=" + std::to_string(point.x) + ", y=" + std::to_string(point.y) +
           ", theta=" + std::to_string(point.angle) + "}";
  };

  return "The path (id " + ipathId + ", length " + std::to_string(length) +
         ") is impossible with waypoints: " +
         std::accumulate(std::next(points.begin()),
                         points.end(),
                         pointToString(points.at(0)),
                         [&](std::string a, Waypoint b) { return a + ", " + pointToString(b); });
}

bool AsyncLinearMotionProfileController::removePath(const std::string &ipathId) {
  if (isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
    LOG_WARN("Attempted to remove currently running path " + ipathId);
    return false;
  }

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    free(oldPath->second.segment);
    paths.erase(ipathId);
  }
  
  // Return true whether the path was actually removed or not
  // Thus, a true return value mirrors the previous behavior of removePath
  return true;
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
  LOG_INFO("AsyncLinearMotionProfileController: Set target to: " + ipathId + " (" +
           std::to_string(ibackwards) + ")");

  currentPath = ipathId;
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
  isRunning.store(true, std::memory_order_release);
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

  while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
    if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
      LOG_INFO("AsyncLinearMotionProfileController: Running with path: " + currentPath);

      auto path = paths.find(currentPath);

      if (path == paths.end()) {
        LOG_WARN(
          "AsyncLinearMotionProfileController: Target was set to non-existent path with name: " +
          currentPath);
      } else {
        LOG_DEBUG("AsyncLinearMotionProfileController: Path length is " +
                  std::to_string(path->second.length));

        executeSinglePath(path->second, timeUtil.getRate());
        output->controllerSet(0);

        LOG_INFO_S("AsyncLinearMotionProfileController: Done moving");
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
    const auto segDT = path.segment[i].dt * second;
    currentProfilePosition = path.segment[i].position;

    const auto motorRPM = convertLinearToRotational(path.segment[i].velocity * mps).convert(rpm);
    output->controllerSet(motorRPM / toUnderlyingType(pair.internalGearset) * reversed);

    rate->delayUntil(segDT);
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
  LOG_INFO_S("AsyncLinearMotionProfileController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("AsyncLinearMotionProfileController: Done waiting to settle");
}

void AsyncLinearMotionProfileController::moveTo(const QLength &iposition,
                                                const QLength &itarget,
                                                bool ibackwards) {
  moveTo(iposition, itarget, limits, ibackwards);
}

void AsyncLinearMotionProfileController::moveTo(const QLength &iposition,
                                                const QLength &itarget,
                                                const PathfinderLimits &ilimits,
                                                const bool ibackwards) {
  static int moveToCount = 0;
  std::string name = "__moveTo" + std::to_string(moveToCount++);
  generatePath({iposition, itarget}, name, ilimits);
  setTarget(name, ibackwards);
  waitUntilSettled();
  if (!removePath(name)) {
    // Failed to remove path (Warn and move on)
    LOG_WARN_S("AsyncLinearMotionProfileController: Couldn't remove path after moveTo");
  }
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

  LOG_INFO_S("AsyncLinearMotionProfileController: Waiting to reset");

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
  LOG_INFO("AsyncLinearMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  disabled.store(iisDisabled, std::memory_order_release);
  // loop() will set the output to 0 when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncLinearMotionProfileController::isDisabled() const {
  return disabled.load(std::memory_order_acquire);
}

void AsyncLinearMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this, "AsyncLinearMotionProfileController");
  }
}

CrossplatformThread *AsyncLinearMotionProfileController::getThread() const {
  return task;
}

void AsyncLinearMotionProfileController::tarePosition() {
}
} // namespace okapi
