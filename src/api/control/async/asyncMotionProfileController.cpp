/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <numeric>

namespace okapi {
AsyncMotionProfileController::AsyncMotionProfileController(
  const TimeUtil &itimeUtil,
  const PathfinderLimits &ilimits,
  const std::shared_ptr<ChassisModel> &imodel,
  const ChassisScales &iscales,
  const AbstractMotor::GearsetRatioPair &ipair,
  const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger),
    limits(ilimits),
    model(imodel),
    scales(iscales),
    pair(ipair),
    timeUtil(itimeUtil) {
  if (ipair.ratio == 0) {
    LOG_ERROR_S("AsyncMotionProfileController: The gear ratio cannot be zero! Check if you are "
                "using integer division.");
    throw std::invalid_argument(
      "AsyncMotionProfileController: The gear ratio cannot be zero! Check "
      "if you are using integer division.");
  }
}

AsyncMotionProfileController::AsyncMotionProfileController(
  AsyncMotionProfileController &&other) noexcept
  : logger(std::move(other.logger)),
    paths(std::move(other.paths)),
    limits(other.limits),
    model(std::move(other.model)),
    scales(other.scales),
    pair(other.pair),
    timeUtil(std::move(other.timeUtil)),
    currentPath(std::move(other.currentPath)),
    isRunning(other.isRunning.load(std::memory_order_acquire)),
    direction(other.direction.load(std::memory_order_acquire)),
    disabled(other.disabled.load(std::memory_order_acquire)),
    dtorCalled(other.dtorCalled.load(std::memory_order_acquire)),
    task(other.task) {
}

AsyncMotionProfileController::~AsyncMotionProfileController() {
  dtorCalled.store(true, std::memory_order_release);

  for (auto &path : paths) {
    free(path.second.left);
    free(path.second.right);
  }

  delete task;
}

void AsyncMotionProfileController::generatePath(std::initializer_list<Point> iwaypoints,
                                                const std::string &ipathId) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    LOG_WARN_S(
      "AsyncMotionProfileController: Not generating a path because no waypoints were given.");

    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      Waypoint{point.x.convert(meter), point.y.convert(meter), point.theta.convert(radian)});
  }

  LOG_INFO_S("AsyncMotionProfileController: Preparing trajectory");

  TrajectoryCandidate candidate;
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
      "AsyncMotionProfileController: The path (length " + std::to_string(length) +
      ") is impossible with waypoints: " +
      std::accumulate(std::next(points.begin()),
                      points.end(),
                      pointToString(points.at(0)),
                      [&](std::string a, Waypoint b) { return a + ", " + pointToString(b); });

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
    std::string message =
      "AsyncMotionProfileController: Could not allocate trajectory. The path (length " +
      std::to_string(length) + ") is probably impossible.";

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  LOG_INFO_S("AsyncMotionProfileController: Generating path");

  pathfinder_generate(&candidate, trajectory);

  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

  if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
    std::string message = "AsyncMotionProfileController: Could not allocate left and/or right "
                          "trajectories. The path (length " +
                          std::to_string(length) + ") is probably impossible.";

    if (leftTrajectory) {
      free(leftTrajectory);
    }

    if (rightTrajectory) {
      free(rightTrajectory);
    }

    if (trajectory) {
      free(trajectory);
    }

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  LOG_INFO_S("AsyncMotionProfileController: Modifying for tank drive");
  pathfinder_modify_tank(
    trajectory, length, leftTrajectory, rightTrajectory, scales.wheelbaseWidth.convert(meter));

  free(trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});

  LOG_INFO_S("AsyncMotionProfileController: Completely done generating path");
  LOG_INFO("AsyncMotionProfileController: Path length: " + std::to_string(length));
}

void AsyncMotionProfileController::removePath(const std::string &ipathId) {
  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    free(oldPath->second.left);
    free(oldPath->second.right);
    paths.erase(ipathId);
  }
}

std::vector<std::string> AsyncMotionProfileController::getPaths() {
  std::vector<std::string> keys;

  for (const auto &path : paths) {
    keys.push_back(path.first);
  }

  return keys;
}

void AsyncMotionProfileController::setTarget(std::string ipathId) {
  setTarget(ipathId, false);
}

void AsyncMotionProfileController::setTarget(std::string ipathId,
                                             const bool ibackwards,
                                             const bool imirrored) {
  currentPath = ipathId;
  isRunning.store(true, std::memory_order_release);
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
  mirrored.store(imirrored, std::memory_order_release);
}

void AsyncMotionProfileController::controllerSet(std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncMotionProfileController::getTarget() {
  return currentPath;
}

void AsyncMotionProfileController::loop() {
  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
    if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
      LOG_INFO("AsyncMotionProfileController: Running with path: " + currentPath);

      auto path = paths.find(currentPath);
      if (path == paths.end()) {
        LOG_WARN("AsyncMotionProfileController: Target was set to non-existent path with name: " +
                 currentPath);
      } else {
        LOG_DEBUG("AsyncMotionProfileController: Path length is " +
                  std::to_string(path->second.length));

        executeSinglePath(path->second, timeUtil.getRate());
        model->stop();

        LOG_INFO_S("AsyncMotionProfileController: Done moving");
      }

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                     std::unique_ptr<AbstractRate> rate) {
  const int reversed = direction.load(std::memory_order_acquire);
  const bool followMirrored = mirrored.load(std::memory_order_acquire);

  if (followMirrored) {
    for (int i = 0; i < path.length && !isDisabled(); ++i) {
      const auto leftRPM = convertLinearToRotational(path.left[i].velocity * mps).convert(rpm);
      const auto rightRPM = convertLinearToRotational(path.right[i].velocity * mps).convert(rpm);

      model->left(rightRPM / toUnderlyingType(pair.internalGearset) * reversed);
      model->right(leftRPM / toUnderlyingType(pair.internalGearset) * reversed);

      rate->delayUntil(1_ms);
    }
  } else {
    for (int i = 0; i < path.length && !isDisabled(); ++i) {
      const auto leftRPM = convertLinearToRotational(path.left[i].velocity * mps).convert(rpm);
      const auto rightRPM = convertLinearToRotational(path.right[i].velocity * mps).convert(rpm);

      model->left(leftRPM / toUnderlyingType(pair.internalGearset) * reversed);
      model->right(rightRPM / toUnderlyingType(pair.internalGearset) * reversed);

      rate->delayUntil(1_ms);
    }
  }
}

QAngularSpeed AsyncMotionProfileController::convertLinearToRotational(QSpeed linear) const {
  return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * pair.ratio;
}

void AsyncMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncMotionProfileController *>(context)->loop();
  }
}

void AsyncMotionProfileController::waitUntilSettled() {
  LOG_INFO_S("AsyncMotionProfileController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("AsyncMotionProfileController: Done waiting to settle");
}

void AsyncMotionProfileController::moveTo(std::initializer_list<Point> iwaypoints,
                                          const bool ibackwards,
                                          const bool imirrored) {
  std::string name = reinterpret_cast<const char *>(this); // hmmmm...
  generatePath(iwaypoints, name);
  setTarget(name, ibackwards, imirrored);
  waitUntilSettled();
  removePath(name);
}

Point AsyncMotionProfileController::getError() const {
  return Point{0_m, 0_m, 0_deg};
}

bool AsyncMotionProfileController::isSettled() {
  return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  auto rate = timeUtil.getRate();
  while (isRunning.load(std::memory_order_acquire)) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncMotionProfileController::flipDisable() {
  flipDisable(!disabled.load(std::memory_order_acquire));
}

void AsyncMotionProfileController::flipDisable(const bool iisDisabled) {
  LOG_INFO("AsyncMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  disabled.store(iisDisabled, std::memory_order_release);
  // loop() will stop the chassis when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncMotionProfileController::isDisabled() const {
  return disabled.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::tarePosition() {
}

void AsyncMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this);
  }
}

CrossplatformThread *AsyncMotionProfileController::getThread() const {
  return task;
}
} // namespace okapi
