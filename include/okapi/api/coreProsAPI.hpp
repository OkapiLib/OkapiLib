#ifndef _OKAPI_COREPROSAPI_HPP_
#define _OKAPI_COREPROSAPI_HPP_

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

  protected:
  CROSSPLATFORM_THREAD_T thread;
};

#endif
