/**
 * @author Kevin Harrington, Common Wealth Robotics
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/control/util/flywheelSimulator.hpp"
#include <cmath>

namespace okapi {
FlywheelSimulator::FlywheelSimulator(const double imass, const double ilinkLen,
                                     const double imuStatic, const double imuDynamic,
                                     const double itimestep)
  : mass(imass),
    linkLen(ilinkLen),
    muStatic(imuStatic),
    muDynamic(imuDynamic),
    timestep(itimestep),
    I(mass * ipow(linkLen, 2)) {
}

FlywheelSimulator::~FlywheelSimulator() = default;

double FlywheelSimulator::step() {
  const double torqueGravity = (linkLen * std::cos(angle)) * (mass * -9.81);
  double torqueTotal = torqueGravity + inputTorque;

  if (omega == 0 && muStatic > std::fabs(torqueTotal)) {
    torqueTotal = 0;
  }

  if (omega != 0) {
    if (torqueTotal > 0) {
      torqueTotal -= muDynamic * omega;
    } else {
      torqueTotal += muDynamic * omega * -1;
    }
  }

  accel = torqueTotal / I;
  omega += accel * ipow(timestep, 2);

  if (omega != 0) {
    angle += omega * timestep;
  }

  if (radianToDegree * angle > 181) {
    angle = pi;
    omega = 0;
  }

  if (radianToDegree * angle < -1) {
    angle = 0;
    omega = 0;
  }

  return angle;
}

void FlywheelSimulator::setTorque(const double itorque) {
  if (std::fabs(itorque) <= std::fabs(maxTorque)) {
    inputTorque = itorque;
  } else {
    inputTorque = std::copysign(maxTorque, itorque);
  }
}

void FlywheelSimulator::setMaxTorque(const double imaxTorque) {
  maxTorque = imaxTorque;
}

void FlywheelSimulator::setMass(const double imass) {
  if (imass < 0) {
    mass = 0;
  } else {
    mass = imass;
  }
}

void FlywheelSimulator::setLinkLength(const double ilinkLen) {
  if (ilinkLen < 0) {
    linkLen = 0;
  } else {
    linkLen = ilinkLen;
  }
}

void FlywheelSimulator::setStaticFriction(const double imuStatic) {
  if (imuStatic < 0) {
    muStatic = 0;
  } else {
    muStatic = imuStatic;
  }
}

void FlywheelSimulator::setDynamicFriction(const double imuDynamic) {
  if (imuDynamic < 0) {
    muDynamic = 0;
  } else {
    muDynamic = imuDynamic;
  }
}

void FlywheelSimulator::setTimestep(const double itimestep) {
  if (timestep < minTimestep) {
    timestep = minTimestep;
  } else {
    timestep = itimestep;
  }
}

double FlywheelSimulator::getAngle() const {
  return angle;
}

double FlywheelSimulator::getOmega() const {
  return omega;
}

double FlywheelSimulator::getAcceleration() const {
  return accel;
}
}
