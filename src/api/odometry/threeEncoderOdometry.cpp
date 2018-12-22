/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include <math.h>

namespace okapi {
ThreeEncoderOdometry::ThreeEncoderOdometry(std::shared_ptr<ReadOnlyChassisModel> imodel,
                                           const ChassisScales &ichassisScales,
                                           const TimeUtil &itimeUtil,
                                           const std::shared_ptr<Logger> &ilogger)
  : Odometry(imodel, ichassisScales, itimeUtil, ilogger),
    logger(ilogger),
    model(imodel),
    rate(itimeUtil.getRate()) {
  if (ichassisScales.middle == 0) {
    logger->error("ThreeEncoderOdometry: Middle scale cannot be zero.");
    throw std::invalid_argument("ThreeEncoderOdometry: Middle scale cannot be zero.");
  }

  if (ichassisScales.middleWheelDistance == 0_m) {
    logger->error("ThreeEncoderOdometry: Middle wheel distance cannot be zero.");
    throw std::invalid_argument("ThreeEncoderOdometry: Middle wheel distance cannot be zero.");
  }
}

void ThreeEncoderOdometry::step() {
  const auto deltaT = timer->getDt();

  if (deltaT.getValue() != 0) {
    newTicks = model->getSensorVals();
    tickDiff = newTicks - lastTicks;
    lastTicks = newTicks;

    const auto Sl = (tickDiff[0] / chassisScales.straight) * meter;
    const auto Sr = (tickDiff[1] / chassisScales.straight) * meter;
    const auto Sm = (tickDiff[2] / chassisScales.middle) * meter;
    const auto Vl = Sl / deltaT;
    const auto Vr = Sr / deltaT;
    const auto b = chassisScales.wheelbaseWidth;
    const auto mB = chassisScales.middleWheelDistance;
    auto turnRadius = (b * (Vr + Vl)) / (2 * (Vr - Vl));

    QLength deltaX;
    QLength deltaY;
    QAngle deltaTheta;
    double sinTheta;
    double cosTheta;

    if ((Vr - Vl).abs() < 0.0001_mps) {
      turnRadius = (Sr + Sl) / 2;
      deltaTheta = 0_deg;

      sinTheta = std::sin(state.theta.convert(radian));
      cosTheta = std::cos(state.theta.convert(radian));
    } else {
      deltaTheta = (((Vr - Vl) * deltaT) / b) * radian;

      if (isnan(deltaTheta.getValue())) {
        deltaTheta = 0_deg;
      }

      sinTheta = std::sin((Vr - Vl).convert(mps) * deltaT.convert(second) / b.convert(meter));
      cosTheta = std::cos((Vr - Vl).convert(mps) * deltaT.convert(second) / b.convert(meter)) - 1;
    }

    const auto deltaMiddle = Sm - ((deltaTheta / 360_deg) * 1_pi * mB);
    deltaX = turnRadius * sinTheta + deltaMiddle * cosTheta;
    deltaY = turnRadius * cosTheta + deltaMiddle * sinTheta;

    if (isnan(deltaX.getValue())) {
      deltaX = 0_m;
    }

    if (isnan(deltaY.getValue())) {
      deltaY = 0_m;
    }

    state.x += deltaX;
    state.y += deltaY;
    state.theta += deltaTheta;
  }
}

void ThreeEncoderOdometry::trampoline(void *context) {
  static_cast<ThreeEncoderOdometry *>(context)->loop();
}
} // namespace okapi
