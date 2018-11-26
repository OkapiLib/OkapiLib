/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>

#ifdef THREADS_STD
#include <thread>
#define CROSSPLATFORM_THREAD_T std::thread
#else
#include "api.h"
#include "pros/apix.h"
#define CROSSPLATFORM_THREAD_T pros::task_t
#endif

class CrossplatformThread {
  public:
  CrossplatformThread(void (*ptr)(void *), void *params)
    :
#ifdef THREADS_STD
      thread(ptr, params)
#else
      thread(pros::c::task_create(ptr,
                                  params,
                                  TASK_PRIORITY_DEFAULT,
                                  TASK_STACK_DEPTH_DEFAULT,
                                  "OkapiLibCrossplatformTask"))
#endif
  {
  }

  ~CrossplatformThread() {
#ifdef THREADS_STD
    thread.join();
#else
    pros::c::task_delete(thread);
#endif
  }

#ifdef THREADS_STD
  void notifyWhenDeleting(CrossplatformThread *) {
  }
#else
  void notifyWhenDeleting(CrossplatformThread *parent) {
    pros::task_notify_when_deleting(parent->thread, thread, 1, pros::E_NOTIFY_ACTION_INCR);
  }
#endif

#ifdef THREADS_STD
  void notifyWhenDeletingRaw(CROSSPLATFORM_THREAD_T *) {
  }
#else
  void notifyWhenDeletingRaw(CROSSPLATFORM_THREAD_T *parent) {
    task_notify_when_deleting(parent, thread, 1, pros::E_NOTIFY_ACTION_INCR);
  }
#endif

#ifdef THREADS_STD
  std::uint32_t notifyTake(const std::uint32_t) {
    return 0;
  }
#else
  std::uint32_t notifyTake(const std::uint32_t itimeout) {
    return pros::c::task_notify_take(true, itimeout);
  }
#endif

  protected:
  CROSSPLATFORM_THREAD_T thread;
};
