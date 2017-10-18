#ifndef OKAPI_PROFILE
#define OKAPI_PROFILE

#include <vector>

namespace okapi {
  class MPTarget {
  public:
    MPTarget(const float velocity, const float acceleration):
      vel(velocity),
      accel(acceleration) {}

    MPTarget():
      vel(0),
      accel(0) {}

    virtual ~MPTarget() = default;

    float vel, accel;
  };

  class MotionProfile {
  public:
    MotionProfile(const float idistance, const float idt):
      data(),
      distance(idistance),
      dt(idt) {}

    virtual ~MotionProfile() { delete &data; }

    MPTarget& operator[](const int index) { return data[index]; }
    
    const MPTarget& operator[](const int index) const { return data[index]; }

    int size() const { return data.size(); }

    std::vector<MPTarget> data;
    float distance, dt;
  };
}

#endif /* end of include guard: OKAPI_PROFILE */
