#include <cmath>
#include "motionProfile/mpGenerator.h"

namespace okapi {
    MotionProfile MPGenerator::generateProfile(const float idt) {
        MotionProfile profile(targetPos, idt);
        float time = 0;
        while (!isCompleteFlag) {
            profile.data.push_back(getNextVelTarget(time));
            time += idt;
        }
        // profile.data.shrink_to_fit();
        return profile;
    }

    MPTarget MPGenerator::getNextVelTarget(const float itime) {
        const float dir = 1 - 2 * static_cast<int>(startVel > maxVel);
        const float velAtExchange = startVel + maxAccel * exchangeTime;

        if (velAtExchange * dir < maxVel * dir)
            return getVelWithoutMaxVel(itime);

        return getVelWithMaxVel(itime);
    }

    float MPGenerator::determineExchangeTime(const float itarget) const {
        const float a = maxAccel * minAccel - maxAccel * maxAccel;
        const float b = 2 * startVel * minAccel - 2 * startVel * maxAccel;
        const float c = endVel * endVel - startVel * startVel - itarget * 2 * minAccel;
        const float squareTerm = b * b - 4 * a * c;

        if (squareTerm < 0) {
            // printf("ERROR: Cannot reach location\n");
            return 0;
        }

        if (a == 0)
            return 0;

        const float temp = std::sqrt(squareTerm);
        const float option1 = (-b + temp) / (2 * a);
        const float option2 = (-b - temp) / (2 * a);

        if (option1 > 0 && option2 < 0)
            return option1;
        if (option2 > 0 && option1 < 0)
            return option2;
        if (option1 < 0 && option2 < 0) {
            // printf("ERROR: Both time results are negative\n");
            return 0;
        }

        return option1;
    }

    MPTarget MPGenerator::getVelWithoutMaxVel(const float itime) {
        if (exchangeTime > itime) {
            return MPTarget(startVel + maxAccel * itime, maxAccel);
        }

        const float velAtExchange = startVel + maxAccel * exchangeTime;
        const float timeToDecel = (endVel - velAtExchange) / minAccel;
        if (itime > timeToDecel + exchangeTime) {
            isCompleteFlag = true;
            return MPTarget(endVel, minAccel);
        }

        return MPTarget(velAtExchange + minAccel * (itime - exchangeTime), minAccel);
    }

    MPTarget MPGenerator::getVelWithMaxVel(const float itime) {
        float timeToMaxVel = 0;
        if (maxAccel != 0)
            timeToMaxVel = (maxVel - startVel) / maxAccel;

        const float distFromAccel = startVel * timeToMaxVel + 0.5f * maxAccel * timeToMaxVel * timeToMaxVel;
        const float distFromMaxVel = maxVel * (itime - timeToMaxVel);

        float timeToStop = 0;
        if (minAccel != 0)
            timeToStop = (endVel - maxVel) / minAccel;

        const float distToStop = maxVel * timeToStop + 0.5f * minAccel * timeToStop * timeToStop;

        const float distAtMaxVel = targetPos - distFromAccel - distToStop;
        const float timeWithMax = distAtMaxVel / maxVel;

        const float dir = 1 - 2 * static_cast<float>(startVel > maxVel || endVel > maxVel);

        if (timeToMaxVel < itime) {
            if (dir * (distFromAccel + distFromMaxVel + distToStop) < targetPos * dir) {
                return MPTarget(maxVel, 0);
            }

            if (itime > timeToMaxVel + timeWithMax + timeToStop) {
                isCompleteFlag = true;
                return MPTarget(endVel, minAccel);
            }

            return MPTarget(maxVel + minAccel * (itime - timeWithMax - timeToMaxVel), minAccel);
        }

        return MPTarget(startVel + maxAccel * itime, maxAccel);
    }
}
