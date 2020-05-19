/*
 * MIT License
 *
 * Copyright (c) 2020 Mapless AI, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef A2D2_TO_ROS__LOG_BUILD_OPTIONS_HPP_
#define A2D2_TO_ROS__LOG_BUILD_OPTIONS_HPP_

#include "a2d2_to_ros/logging.hpp"

#ifdef ENABLE_A2D2_ROS_LOGGING
#define LOGGING_MSG X_INFO("---Built with ROS logging enabled.")
#elif defined(ENABLE_A2D2_STREAM_LOGGING)
#define LOGGING_MSG X_INFO("---Built with stream logging enabled.")
#else
#define LOGGING_MSG
#endif

#ifdef USE_FLOAT64
#define PRECISION_MSG                                                         \
  X_INFO(                                                                     \
      "---Built to use double precision for float values; be aware this may " \
      "break compatibility with Rviz.")
#else
#define PRECISION_MSG \
  X_INFO("---Built to use single precision for float values.")
#endif

#define BUILD_INFO \
  LOGGING_MSG;     \
  PRECISION_MSG;

#endif  // A2D2_TO_ROS__LOG_BUILD_OPTIONS_HPP_
