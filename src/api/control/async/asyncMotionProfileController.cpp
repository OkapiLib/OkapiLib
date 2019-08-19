/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>
#include <iostream>
#include <mutex>
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
    std::string msg("AsyncMotionProfileController: The gear ratio cannot be zero! Check if you are "
                    "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
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
  generatePath(iwaypoints, ipathId, limits);
}

void AsyncMotionProfileController::generatePath(std::initializer_list<Point> iwaypoints,
                                                const std::string &ipathId,
                                                const PathfinderLimits &ilimits) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    LOG_WARN(std::string(
      "AsyncMotionProfileController: Not generating a path because no waypoints were given."));
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      Waypoint{point.x.convert(meter), point.y.convert(meter), point.theta.convert(radian)});
  }

  LOG_INFO(std::string("AsyncMotionProfileController: Preparing trajectory"));

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
    std::string message = "AsyncMotionProfileController: Length was negative. " +
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
    std::string message = "AsyncMotionProfileController: Could not allocate trajectory. " +
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

  LOG_INFO(std::string("AsyncMotionProfileController: Generating path"));

  pathfinder_generate(&candidate, trajectory);

  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

  if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
    std::string message = "AsyncMotionProfileController: Could not allocate left and/or right "
                          "trajectories. " +
                          getPathErrorMessage(points, ipathId, length);

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

  LOG_INFO(std::string("AsyncMotionProfileController: Modifying for tank drive"));
  pathfinder_modify_tank(
    trajectory, length, leftTrajectory, rightTrajectory, scales.wheelTrack.convert(meter));

  free(trajectory);

  // Free the old path before overwriting it
  forceRemovePath(ipathId);

  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});

  LOG_INFO("AsyncMotionProfileController: Completely done generating path " + ipathId);
  LOG_DEBUG("AsyncMotionProfileController: Path length: " + std::to_string(length));
}

std::string AsyncMotionProfileController::getPathErrorMessage(const std::vector<Waypoint> &points,
                                                              const std::string &ipathId,
                                                              int length) {
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

bool AsyncMotionProfileController::removePath(const std::string &ipathId) {
  if (!isDisabled() && isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
    LOG_WARN("AsyncMotionProfileController: Attempted to remove currently running path " + ipathId);
    return false;
  }

  std::scoped_lock lock(currentPathMutex);

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    free(oldPath->second.left);
    free(oldPath->second.right);
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
  LOG_INFO("AsyncMotionProfileController: Set target to: " + ipathId + " (" +
           std::to_string(ibackwards) + ", " + std::to_string(imirrored) + ")");

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

        LOG_INFO(std::string("AsyncMotionProfileController: Done moving"));
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
  const int pathLength = getPathLength(path);

  for (int i = 0; i < pathLength && !isDisabled(); ++i) {
    // This mutex is used to combat an edge case of an edge case
    // if a running path is asked to be removed at the moment this loop is executing
    std::scoped_lock lock(currentPathMutex);

    const auto segDT = path.left[i].dt * second;
    const auto leftRPM = convertLinearToRotational(path.left[i].velocity * mps).convert(rpm);
    const auto rightRPM = convertLinearToRotational(path.right[i].velocity * mps).convert(rpm);

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

int AsyncMotionProfileController::getPathLength(const TrajectoryPair &path) {
  std::scoped_lock lock(currentPathMutex);
  return path.length;
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
  LOG_INFO(std::string("AsyncMotionProfileController: Waiting to settle"));

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  LOG_INFO(std::string("AsyncMotionProfileController: Done waiting to settle"));
}

void AsyncMotionProfileController::moveTo(std::initializer_list<Point> iwaypoints,
                                          bool ibackwards,
                                          bool imirrored) {
  moveTo(iwaypoints, limits, ibackwards, imirrored);
}

void AsyncMotionProfileController::moveTo(std::initializer_list<Point> iwaypoints,
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

Point AsyncMotionProfileController::getError() const {
  return Point{0_m, 0_m, 0_deg};
}

bool AsyncMotionProfileController::isSettled() {
  return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  LOG_INFO(std::string("AsyncMotionProfileController: Waiting to reset"));

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
    task = new CrossplatformThread(trampoline, this, "AsyncMotionProfileController");
  }
}

CrossplatformThread *AsyncMotionProfileController::getThread() const {
  return task;
}

void AsyncMotionProfileController::storePath(const std::string &idirectory,
                                             const std::string &ipathId) {
  std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
  std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
  FILE *leftPathFile = fopen(leftFilePath.c_str(), "w");
  FILE *rightPathFile = fopen(rightFilePath.c_str(), "w");

  // Make sure we can open the file successfully
  if (leftPathFile == NULL) {
    LOG_WARN("AsyncMotionProfileController: Couldn't open file " + leftFilePath + " for writing");
    if (rightPathFile != NULL) {
      fclose(rightPathFile);
    }
    return;
  }
  if (rightPathFile == NULL) {
    LOG_WARN("AsyncMotionProfileController: Couldn't open file " + rightFilePath + " for writing");
    fclose(leftPathFile);
    return;
  }

  internalStorePath(leftPathFile, rightPathFile, ipathId);

  fclose(leftPathFile);
  fclose(rightPathFile);
}

void AsyncMotionProfileController::loadPath(const std::string &idirectory,
                                            const std::string &ipathId) {
  std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
  std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
  FILE *leftPathFile = fopen(leftFilePath.c_str(), "r");
  FILE *rightPathFile = fopen(rightFilePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (leftPathFile == NULL) {
    LOG_WARN("AsyncMotionProfileController: Couldn't open file " + leftFilePath + " for reading");
    if (rightPathFile != NULL) {
      fclose(rightPathFile);
    }
    return;
  }
  if (rightPathFile == NULL) {
    LOG_WARN("AsyncMotionProfileController: Couldn't open file " + rightFilePath + " for reading");
    fclose(leftPathFile);
    return;
  }

  internalLoadPath(leftPathFile, rightPathFile, ipathId);

  fclose(leftPathFile);
  fclose(rightPathFile);
}

void AsyncMotionProfileController::internalStorePath(FILE *leftPathFile,
                                                     FILE *rightPathFile,
                                                     const std::string &ipathId) {
  auto pathData = this->paths.find(ipathId);

  // Make sure path exists
  if (pathData == paths.end()) {
    LOG_WARN("AsyncMotionProfileController: Controller was asked to serialize non-existent path " +
             ipathId);
    // Do nothing- can't serialize nonexistent path
  } else {
    int len = pathData->second.length;

    // Serialize paths
    pathfinder_serialize_csv(leftPathFile, pathData->second.left, len);
    pathfinder_serialize_csv(rightPathFile, pathData->second.right, len);
  }
}

void AsyncMotionProfileController::internalLoadPath(FILE *leftPathFile,
                                                    FILE *rightPathFile,
                                                    const std::string &ipathId) {
  // Count lines in file, remove one for headers
  int count = 0;
  for (int c = getc(leftPathFile); c != EOF; c = getc(leftPathFile)) {
    if (c == '\n') {
      ++count;
    }
  }
  --count;
  rewind(leftPathFile);

  // Allocate memory
  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * count);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * count);

  pathfinder_deserialize_csv(leftPathFile, leftTrajectory);
  pathfinder_deserialize_csv(rightPathFile, rightTrajectory);

  // Remove the old path if it exists
  forceRemovePath(ipathId);
  paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, count});
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
