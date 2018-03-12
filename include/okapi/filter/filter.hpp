/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_FILTER_HPP_
#define _OKAPI_FILTER_HPP_

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

#endif
