#include "odometry/odomMath.h"
#include <cmath>
#include "util/mathUtil.h"

namespace okapi {
  float OdomMath::computeDistanceToPoint(const float ix, const float iy, const OdomState& istate) {
    const float xDiff = ix - istate.x;
    const float yDiff = iy - istate.y;
    return std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
  }

  float OdomMath::computeAngleToPoint(const float ix, const float iy, const OdomState& istate) {
    const float xDiff = ix - istate.x;
    const float yDiff = iy - istate.y;
    return (std::atan2(yDiff, xDiff) * radianToDegree) - istate.theta;
  }

  DistanceAndAngle OdomMath::computeDistanceAndAngleToPoint(const float ix, const float iy, const OdomState& istate) {
    using namespace std; //Needed to get copysign to compile

    const float xDiff = ix - istate.x;
    const float yDiff = iy - istate.y;
    DistanceAndAngle out;
    out.length = std::sqrt((xDiff * xDiff) + (yDiff * yDiff));

    //Small xDiff is essentially dividing by zero, so avoid it and do custom math
    if (xDiff < 0.0001 && xDiff > -0.0001) {
      const int yDiffSign = static_cast<int>(copysign(1, yDiff));
      if (yDiffSign == 1) {
        out.theta = -1 * istate.theta;
      } else if (yDiffSign == -1) {
        out.theta = -180 - istate.theta;

        //Fix theta
        if (out.theta <= -360)
          out.theta += 360;
        else if (out.theta >= 360)
          out.theta -= 360;
      }
    } else {
      out.theta = (std::atan2(yDiff, xDiff) * radianToDegree) - istate.theta;
    }

    return out;
  }
}
