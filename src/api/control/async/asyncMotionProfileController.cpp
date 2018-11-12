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
  const double imaxVel,
  const double imaxAccel,
  const double imaxJerk,
  const std::shared_ptr<ChassisModel> &imodel,
  const ChassisScales &iscales,
  AbstractMotor::GearsetRatioPair ipair)
  : logger(Logger::instance()),
    maxVel(imaxVel),
    maxAccel(imaxAccel),
    maxJerk(imaxJerk),
    model(imodel),
    scales(iscales),
    pair(ipair),
    timeUtil(itimeUtil) {
  if (ipair.ratio == 0) {
    logger->error("AsyncMotionProfileController: The gear ratio cannot be zero! Check if you are "
                  "using integer division.");
    throw std::invalid_argument(
      "AsyncMotionProfileController: The gear ratio cannot be zero! Check "
      "if you are using integer division.");
  }
}

AsyncMotionProfileController::AsyncMotionProfileController(
  AsyncMotionProfileController &&other) noexcept
  : logger(other.logger),
    paths(std::move(other.paths)),
    maxVel(other.maxVel),
    maxAccel(other.maxAccel),
    maxJerk(other.maxJerk),
    model(std::move(other.model)),
    scales(other.scales),
    pair(other.pair),
    timeUtil(std::move(other.timeUtil)),
    currentPath(std::move(other.currentPath)),
    isRunning(other.isRunning.load(std::memory_order_acquire)),
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
      "AsyncMotionProfileController: Path is impossible with waypoints: " +
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
    std::string message = "AsyncMotionProfileController: Could not allocate trajectory. The path "
                          "is probably impossible.";
    logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  logger->info("AsyncMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

  if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
    std::string message = "AsyncMotionProfileController: Could not allocate left and/or right "
                          "trajectories. The path is probably impossible.";
    logger->error(message);

    if (leftTrajectory) {
      free(leftTrajectory);
    }

    if (rightTrajectory) {
      free(rightTrajectory);
    }

    if (trajectory) {
      free(trajectory);
    }

    throw std::runtime_error(message);
  }

  logger->info("AsyncMotionProfileController: Modifying for tank drive");
  pathfinder_modify_tank(
    trajectory, length, leftTrajectory, rightTrajectory, scales.wheelbaseWidth.convert(meter));

  free(trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});
  logger->info("AsyncMotionProfileController: Completely done generating path");
  logger->info("AsyncMotionProfileController: " + std::to_string(length));
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

void AsyncMotionProfileController::setTarget(std::string ipathId, const bool ibackwards) {
  currentPath = ipathId;
  isRunning.store(true, std::memory_order_release);
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
}

void AsyncMotionProfileController::controllerSet(std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncMotionProfileController::getTarget() {
  return currentPath;
}

void AsyncMotionProfileController::loop() {
  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order_acquire)) {
    if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
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

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                     std::unique_ptr<AbstractRate> rate) {
  const auto reversed = direction.load(std::memory_order_acquire);

  for (int i = 0; i < path.length && !isDisabled(); ++i) {
    const auto leftRPM = convertLinearToRotational(path.left[i].velocity * mps).convert(rpm);
    const auto rightRPM = convertLinearToRotational(path.right[i].velocity * mps).convert(rpm);

    model->left(leftRPM / toUnderlyingType(pair.internalGearset) * reversed);
    model->right(rightRPM / toUnderlyingType(pair.internalGearset) * reversed);

    rate->delayUntil(1_ms);
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
  logger->info("AsyncMotionProfileController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  logger->info("AsyncMotionProfileController: Done waiting to settle");
}

void AsyncMotionProfileController::moveTo(std::initializer_list<Point> iwaypoints) {
  std::string name = reinterpret_cast<const char *>(this); // hmmmm...
  generatePath(iwaypoints, name);
  setTarget(name);
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
  logger->info("AsyncMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  disabled.store(iisDisabled, std::memory_order_release);
  // loop() will stop the chassis when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncMotionProfileController::isDisabled() const {
  return disabled.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this);
  }
}
} // namespace okapi
