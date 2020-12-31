/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include <algorithm>
#include <iostream>
#include <fstream>
#include <mutex>
#include <numeric>

#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"

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
    std::string msg("AsyncMotionProfileController: The gear ratio cannot be zero! Check if you are "
                    "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}

AsyncMotionProfileController::~AsyncMotionProfileController() {
  dtorCalled.store(true, std::memory_order_release);

  // Free paths before deleting the task
  std::scoped_lock lock(currentPathMutex);
  paths.clear();

  delete task;
}

void AsyncMotionProfileController::generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                                                const std::string &ipathId) {
  generatePath(iwaypoints, ipathId, limits);
}

void AsyncMotionProfileController::generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                                                const std::string &ipathId,
                                                const PathfinderLimits &ilimits) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    LOG_WARN_S(
      "AsyncMotionProfileController: Not generating a path because no waypoints were given.");
    return;
  }

  std::vector<squiggles::Pose> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      squiggles::Pose{point.y.convert(meter),
                      point.x.convert(meter),
                      (90_deg - point.theta).convert(radian)});
  }

  LOG_INFO_S("AsyncMotionProfileController: Preparing trajectory");

  auto constraints = squiggles::Constraints(ilimits.maxVel, ilimits.maxAccel, ilimits.maxJerk);
  auto splineGenerator = squiggles::SplineGenerator(constraints, 
    std::make_shared<squiggles::TankModel>(scales.wheelTrack.convert(meter), constraints), 
    DT);
  auto path = splineGenerator.generate(points);


  // Free the old path before overwriting it
  forceRemovePath(ipathId);

  paths.insert({ipathId, path});

  LOG_INFO("AsyncMotionProfileController: Completely done generating path " + ipathId);
  LOG_DEBUG("AsyncMotionProfileController: Path length: " + std::to_string(path.size()));
}

std::string AsyncMotionProfileController::getPathErrorMessage(const std::vector<PathfinderPoint> &points,
                                                              const std::string &ipathId,
                                                              int length) {
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

bool AsyncMotionProfileController::removePath(const std::string &ipathId) {
  if (!isDisabled() && isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
    LOG_WARN("AsyncMotionProfileController: Attempted to remove currently running path " + ipathId);
    return false;
  }

  std::scoped_lock lock(currentPathMutex);

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    paths.erase(ipathId);
  }

  // A return value of true provides no feedback about whether the path was actually removed but
  // instead tells us that the path does not exist at this moment
  return true;
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
  LOG_INFO("AsyncMotionProfileController: Set target to: " + ipathId + " (ibackwards=" +
           std::to_string(ibackwards) + ", imirrored=" + std::to_string(imirrored) + ")");

  currentPath = ipathId;
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
  mirrored.store(imirrored, std::memory_order_release);
  isRunning.store(true, std::memory_order_release);
}

void AsyncMotionProfileController::controllerSet(std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncMotionProfileController::getTarget() {
  return currentPath;
}

std::string AsyncMotionProfileController::getProcessValue() const {
  return currentPath;
}

void AsyncMotionProfileController::loop() {
  LOG_INFO_S("Started AsyncMotionProfileController task.");

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
                  std::to_string(path->second.size()));

        executeSinglePath(path->second, timeUtil.getRate());

        // Stop the chassis after the path because:
        // 1. We only support an exit velocity of zero
        // 2. Because of (1), we should make sure the system is stopped
        model->stop();

        LOG_INFO_S("AsyncMotionProfileController: Done moving");
      }

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("Stopped AsyncMotionProfileController task.");
}

void AsyncMotionProfileController::executeSinglePath(const std::vector<squiggles::ProfilePoint> &path,
                                                     std::unique_ptr<AbstractRate> rate) {
  const int reversed = direction.load(std::memory_order_acquire);
  const bool followMirrored = mirrored.load(std::memory_order_acquire);

  currentPathMutex.lock();
  // store this locally so we aren't accessing the path when we don't know if it's valid
  std::size_t pathSize = path.size(); 
  currentPathMutex.unlock();
  for (std::size_t i = 0; i < pathSize && !isDisabled(); ++i) {
    // This mutex is used to combat an edge case of an edge case
    // if a running path is asked to be removed at the moment this loop is executing
    std::scoped_lock lock(currentPathMutex);

    const auto segDT = DT * second;
    const auto leftRPM = convertLinearToRotational(path[i].wheel_velocities[0] * mps).convert(rpm);
    const auto rightRPM =
      convertLinearToRotational(path[i].wheel_velocities[1] * mps).convert(rpm);

    const double rightSpeed = rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
    const double leftSpeed = leftRPM / toUnderlyingType(pair.internalGearset) * reversed;
    if (followMirrored) {
      model->left(rightSpeed);
      model->right(leftSpeed);
    } else {
      model->left(leftSpeed);
      model->right(rightSpeed);
    }

    // Unlock before the delay to be nice to other tasks
    currentPathMutex.unlock();

    rate->delayUntil(segDT);
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

void AsyncMotionProfileController::moveTo(std::initializer_list<PathfinderPoint> iwaypoints,
                                          bool ibackwards,
                                          bool imirrored) {
  moveTo(iwaypoints, limits, ibackwards, imirrored);
}

void AsyncMotionProfileController::moveTo(std::initializer_list<PathfinderPoint> iwaypoints,
                                          const PathfinderLimits &ilimits,
                                          const bool ibackwards,
                                          const bool imirrored) {
  static int moveToCount = 0;
  std::string name = "__moveTo" + std::to_string(moveToCount++);
  generatePath(iwaypoints, name, ilimits);
  setTarget(name, ibackwards, imirrored);
  waitUntilSettled();
  forceRemovePath(name);
}

PathfinderPoint AsyncMotionProfileController::getError() const {
  return PathfinderPoint{0_m, 0_m, 0_deg};
}

bool AsyncMotionProfileController::isSettled() {
  return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  LOG_INFO_S("AsyncMotionProfileController: Waiting to reset");

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

void AsyncMotionProfileController::setMaxVelocity(std::int32_t) {
}

void AsyncMotionProfileController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this, "AsyncMotionProfileController");
  }
}

CrossplatformThread *AsyncMotionProfileController::getThread() const {
  return task;
}

void AsyncMotionProfileController::storePath(const std::string &idirectory,
                                             const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ipathId + ".csv");
  std::ofstream file;
  file.open(filePath, std::ofstream::out);

  // Make sure we can open the file successfully
  if (!file.good()) {
    LOG_WARN("AsyncMotionProfileController: Couldn't open file " + filePath + " for writing");
    return;
  }

  internalStorePath(file, ipathId);

  file.close();
}

void AsyncMotionProfileController::loadPath(const std::string &idirectory,
                                            const std::string &ipathId) {
  std::string squigglesPath = makeFilePath(idirectory, ipathId + ".csv");  
  std::ifstream squigglesPathFile;
  squigglesPathFile.open(squigglesPath, std::ifstream::in);
  if (squigglesPathFile.good()) {
    // give preference to a squiggles path stored for this id
    internalLoadPath(squigglesPathFile, ipathId);
    squigglesPathFile.close();
    return;
  }

  // There's no Squiggles path, let's check for Pathfinder files
  std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
  std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
  std::ifstream leftPathFile, rightPathFile;
  leftPathFile.open(leftFilePath, std::ifstream::in);
  rightPathFile.open(rightFilePath, std::ifstream::in);
  if (leftPathFile.good() && rightPathFile.good()) {
    internalLoadPathfinderPath(leftPathFile, rightPathFile, ipathId);
    leftPathFile.close();
    rightPathFile.close();
  } else {
    // we don't have both pathfinder files available, check if there's one
    if (rightPathFile.good()) {
      LOG_WARN("AsyncMotionProfileController: Couldn't open file " + leftFilePath + " for reading");
      rightPathFile.close();
      return;
    } 
    if (leftPathFile.good()) {
      LOG_WARN("AsyncMotionProfileController: Couldn't open file " + rightFilePath + " for reading");
      leftPathFile.close();
      return;
    }
    LOG_WARN("AsyncMotionProfileController: Couldn't find any path files for id " + ipathId);
  }
}

void AsyncMotionProfileController::internalStorePath(std::ostream &file,
                                                     const std::string &ipathId) {
  auto pathData = this->paths.find(ipathId);

  // Make sure path exists
  if (pathData == paths.end()) {
    LOG_WARN("AsyncMotionProfileController: Controller was asked to serialize non-existent path " +
             ipathId);
    // Do nothing- can't serialize nonexistent path
  } else {
    squiggles::serialize_path(file, pathData->second);
  }
}

void AsyncMotionProfileController::internalLoadPath(std::istream &file,
                                                    const std::string &ipathId) {

  auto path = squiggles::deserialize_path(file);
  forceRemovePath(ipathId);
  paths.emplace(ipathId, path.value());
}

void AsyncMotionProfileController::internalLoadPathfinderPath(std::istream &leftFile,
                                                              std::istream &rightFile,
                                                    const std::string &ipathId) {

  auto path = squiggles::deserialize_pathfinder_path(leftFile, rightFile);
  forceRemovePath(ipathId);
  paths.emplace(ipathId, path.value());
}

std::string AsyncMotionProfileController::makeFilePath(const std::string &directory,
                                                       const std::string &filename) {
  std::string path(directory);

  // Checks first substring
  if (path.rfind("/usd", 0) == std::string::npos) {
    if (path.rfind("usd", 0) != std::string::npos) {
      // There's a usd, but no beginning slash
      path.insert(0, "/"); // We just need a slash
    } else {               // There's nothing at all
      if (path.front() == '/') {
        // Don't double up on slashes
        path.insert(0, "/usd");
      } else {
        path.insert(0, "/usd/");
      }
    }
  }

  // Add trailing slash if there isn't one
  if (path.back() != '/') {
    path.append("/");
  }
  std::string filenameCopy(filename);
  // Remove restricted characters from filename
  static const std::string illegalChars = "\\/:?*\"<>|";
  for (auto it = filenameCopy.begin(); it < filenameCopy.end(); it++) {
    if (illegalChars.rfind(*it) != std::string::npos) {
      it = filenameCopy.erase(it);
    }
  }

  path.append(filenameCopy);

  return path;
}

void AsyncMotionProfileController::forceRemovePath(const std::string &ipathId) {
  if (!removePath(ipathId)) {
    LOG_WARN("AsyncMotionProfileController: Disabling controller to remove path " + ipathId);
    flipDisable(true);
    removePath(ipathId);
  }
}
} // namespace okapi
