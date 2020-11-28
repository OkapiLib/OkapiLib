/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <mutex>
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
  if (ipair.ratio == 0) {
    std::string msg(
      "AsyncLinearMotionProfileController: The gear ratio cannot be zero! Check if you are "
      "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}

AsyncLinearMotionProfileController::~AsyncLinearMotionProfileController() {
  dtorCalled.store(true, std::memory_order_release);

  // Free paths before deleting the task
  std::scoped_lock lock(currentPathMutex);
  paths.clear();

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
    LOG_WARN_S("AsyncLinearMotionProfileController: Not generating a path because no "
               "waypoints were given.");
    return;
  }

  std::vector<squiggles::Pose> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      squiggles::Pose{point.convert(meter), 0, 0});
  }

  LOG_INFO_S("AsyncLinearMotionProfileController: Preparing trajectory");

  auto constraints = squiggles::Constraints(ilimits.maxVel, ilimits.maxAccel, ilimits.maxJerk);
  auto splineGenerator = squiggles::SplineGenerator(constraints, 
    std::make_shared<squiggles::PassthroughModel>(), 
    0.01);
  auto path = splineGenerator.generate(points);


  // Free the old path before overwriting it
  forceRemovePath(ipathId);

  paths.emplace(ipathId, path);

  LOG_INFO("AsyncLinearMotionProfileController: Completely done generating path " + ipathId);
  LOG_DEBUG("AsyncLinearMotionProfileController: Path length: " + std::to_string(path.size()));
}

std::string
AsyncLinearMotionProfileController::getPathErrorMessage(const std::vector<PathfinderPoint> &points,
                                                        const std::string &ipathId,
                                                        const int length) {
  auto pointToString = [](PathfinderPoint point) {
    return "PathfinderPoint{x=" + std::to_string(point.x.getValue()) + ", y=" + std::to_string(point.y.getValue()) +
           ", theta=" + std::to_string(point.theta.getValue()) + "}";
  };

  return "The path (id " + ipathId + ", length " + std::to_string(length) +
         ") is impossible with waypoints: " +
         std::accumulate(std::next(points.begin()),
                         points.end(),
                         pointToString(points.at(0)),
                         [&](std::string a, PathfinderPoint b) { return a + ", " + pointToString(b); });
}

bool AsyncLinearMotionProfileController::removePath(const std::string &ipathId) {
  if (!isDisabled() && isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
    LOG_WARN("AsyncLinearMotionProfileController: Attempted to remove currently running path " +
             ipathId);
    return false;
  }

  std::scoped_lock lock(currentPathMutex);

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    paths.erase(ipathId);
  }

  /*
   * A return value of true provides no feedback about whether the
   * path was actually removed but instead tells us that the path
   * does not exist at this moment
   */
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
  LOG_INFO("AsyncLinearMotionProfileController: Set target to: " + ipathId + " (ibackwards" +
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

std::string AsyncLinearMotionProfileController::getProcessValue() const {
  return currentPath;
}

void AsyncLinearMotionProfileController::loop() {
  LOG_INFO_S("Started AsyncLinearMotionProfileController task.");

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
                  std::to_string(path->second.size()));

        executeSinglePath(path->second, timeUtil.getRate());

        // Set 0 after the path because:
        // 1. We only support an exit velocity of zero
        // 2. Because of (1), we should make sure the system is stopped
        output->controllerSet(0);

        LOG_INFO_S("AsyncLinearMotionProfileController: Done moving");
      }

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("Stopped AsyncLinearMotionProfileController task.");
}

void AsyncLinearMotionProfileController::executeSinglePath(const std::vector<squiggles::ProfilePoint> &path,
                                                           std::unique_ptr<AbstractRate> rate) {
  const auto reversed = direction.load(std::memory_order_acquire);

  std::scoped_lock pathLock(currentPathMutex);
  // store this locally so we aren't accessing the path when we don't know if it's valid
  std::size_t pathSize = path.size(); 
  currentPathMutex.unlock();
  for (std::size_t i = 0; i < pathSize && !isDisabled(); ++i) {
    // This mutex is used to combat an edge case of an edge case
    // if a running path is asked to be removed at the moment this loop is executing
    std::scoped_lock lock(currentPathMutex);

    const auto segDT = path[i].time * millisecond;
    currentProfilePosition = path[i].vector.pose.x;

    const auto motorRPM =
      convertLinearToRotational(path[i].vector.vel * mps).convert(rpm);
    output->controllerSet(motorRPM / toUnderlyingType(pair.internalGearset) * reversed);

    // Unlock before the delay to be nice to other tasks
    currentPathMutex.unlock();

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
    return path->second.back().vector.pose.x - currentProfilePosition;
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

void AsyncLinearMotionProfileController::setMaxVelocity(std::int32_t) {
}

void AsyncLinearMotionProfileController::forceRemovePath(const std::string &ipathId) {
  if (!removePath(ipathId)) {
    LOG_WARN("AsyncLinearMotionProfileController: Disabling controller to remove path " + ipathId);
    flipDisable(true);
    removePath(ipathId);
  }
}

} // namespace okapi
