#ifndef OKAPI_FILTER
#define OKAPI_FILTER

namespace okapi {
  class Filter {
  public:
    Filter() {}
    virtual ~Filter() = default;
    
    /**
     * Filters a reading
     * @param  reading New measurement
     * @return         Filtered result
     */
    virtual float filter(const float ireading) = 0;

    /**
     * Returns the previous output from filter
     * @return The previous output from filter
     */
    virtual float getOutput() const = 0;
  };
}

#endif /* end of include guard: OKAPI_FILTER */
