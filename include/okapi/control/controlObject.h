#ifndef OKAPI_CONTORLOBJECT
#define OKAPI_CONTORLOBJECT

namespace okapi {
  class ControlObjectParams {};

  class ControlObject {
  public:
    virtual ~ControlObject() = default;

    /**
    * Do one iteration of the controller
    * @param  inewReading New measurement
    * @return            Controller output
    */
    virtual float step(const float ireading) = 0;

    /**
    * Sets the target for the controller
    */
    virtual void setTarget(const float itarget) = 0;

    /**
    * Returns the last calculated output of the controller
    */
    virtual float getOutput() const = 0;

    /**
    * Returns the last error of the controller
    */
    virtual float getError() const = 0;

    /**
    * Set time between loops in ms
    * @param isampleTime Time between loops in ms
    */
    virtual void setSampleTime(const int isampleTime);

    /**
    * Set controller output bounds
    * @param imax Max output
    * @param imin Min output
    */
    virtual void setOutputLimits(float imax, float imin);

    /**
    * Resets the controller so it can start from 0 again properly. Keeps
    * configuration from before
    */
    virtual void reset();

    /**
    * Turns the controller on or off
    */
    virtual void flipDisable();
  };
}

#endif /* OKAPI_CONTROLOBJECT */
