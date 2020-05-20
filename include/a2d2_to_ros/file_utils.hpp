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
#ifndef A2D2_TO_ROS__FILE_UTILS_HPP_
#define A2D2_TO_ROS__FILE_UTILS_HPP_

#include <string>

namespace a2d2_to_ros {

/**
 * @brief Get the frame of the data from its filename.
 * @return The empty string if a frame name is not present or if multiple
 * different names are present.
 */
std::string frame_from_filename(const std::string& filename);

/**
 * @brief Load an entire file into memory as a string.
 * @pre The file pointed to by path is non-empty.
 * @note This function has no test coverage.
 * @return The file text, or an empty string if loading the file failed.
 */
std::string get_file_as_string(const std::string& path);

}  // namespace a2d2_to_ros

#endif  // A2D2_TO_ROS__FILE_UTILS_HPP_
