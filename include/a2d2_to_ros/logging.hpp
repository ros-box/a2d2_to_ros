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
#ifndef A2D2_TO_ROS__LOGGING_HPP_
#define A2D2_TO_ROS__LOGGING_HPP_

#ifdef ENABLE_A2D2_ROS_LOGGING
#include <ros/console.h>
#define X_INFO(s) ROS_INFO_STREAM(s)
#define X_WARN(s) ROS_WARN_STREAM(s)
#define X_ERROR(s) ROS_ERROR_STREAM(s)
#define X_FATAL(s) ROS_FATAL_STREAM(s)
#elif defined(ENABLE_A2D2_STREAM_LOGGING)
#include <iostream>
#define X_INFO(s) std::cout << s << std::endl;
#define X_WARN(s) std::cout << s << std::endl;
#define X_ERROR(s) std::cerr << s << std::endl;
#define X_FATAL(s) std::cerr << s << std::endl;
#else
#define X_INFO(s)
#define X_WARN(s)
#define X_ERROR(s)
#define X_FATAL(s)
#endif

#endif  // A2D2_TO_ROS__LOGGING_HPP_
