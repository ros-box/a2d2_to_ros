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
#include "a2d2_to_ros/file_utils.hpp"

#include <algorithm>
#include <fstream>

#include "a2d2_to_ros/sensors.hpp"

namespace a2d2_to_ros {

//------------------------------------------------------------------------------

std::string frame_from_filename(const std::string& filename) {
  const auto frames = sensors::Frames::get_files();

  const auto filename_has_frame = [&filename](const std::string& frame) {
    return (filename.find(frame) != std::string::npos);
  };

  const auto num_found =
      std::count_if(std::begin(frames), std::end(frames), filename_has_frame);
  if (num_found != 1) {
    return "";
  }

  const auto it =
      std::find_if(std::begin(frames), std::end(frames), filename_has_frame);

  const auto idx = (it - std::begin(frames));
  return frames[idx];
}

//------------------------------------------------------------------------------

std::string get_file_as_string(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.good()) {
    return std::string("");
  }

  try {
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    return content;
  } catch (...) {
    return std::string("");
  }
}

//------------------------------------------------------------------------------

}  // namespace a2d2_to_ros
