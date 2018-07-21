#ifndef _OKAPI_COREPROSAPI_HPP_
#define _OKAPI_COREPROSAPI_HPP_

#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#ifdef THREADS_STD
#include <thread>
#define CROSSPLATFORM_THREAD std::thread
#else
#include "api.h"
#define CROSSPLATFORM_THREAD pros::Task
#endif

#endif
