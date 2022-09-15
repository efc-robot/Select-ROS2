// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcpputils/find_library.hpp"

#include <cassert>
#include <cstddef>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rcutils/filesystem.h"
#include "rcutils/get_env.h"

#include "rcpputils/split.hpp"
#include "rcpputils/get_env.hpp"

namespace rcpputils
{

namespace
{

#ifdef _WIN32
static constexpr char kPathVar[] = "PATH";
static constexpr char kPathSeparator = ';';
static constexpr char kSolibPrefix[] = "";
static constexpr char kSolibExtension[] = ".dll";
#elif __APPLE__
static constexpr char kPathVar[] = "DYLD_LIBRARY_PATH";
static constexpr char kPathSeparator = ':';
static constexpr char kSolibPrefix[] = "lib";
static constexpr char kSolibExtension[] = ".dylib";
#else
static constexpr char kPathVar[] = "LD_LIBRARY_PATH";
static constexpr char kPathSeparator = ':';
static constexpr char kSolibPrefix[] = "lib";
static constexpr char kSolibExtension[] = ".so";
#endif

}  // namespace
//寻找制定库，在dds中，寻找dds的库文件，现在改成的是rmw_cyclone; library = "rmw_cyclone"
std::string find_library_path(const std::string & library_name)
{
  std::string search_path = get_env_var(kPathVar);
  std::vector<std::string> search_paths = rcpputils::split(search_path, kPathSeparator);
  // kSolibPrefix = "lib/"
  std::string filename = kSolibPrefix;
  filename += library_name + kSolibExtension;
  //search_path = "/home/asamu/ros2_foxy_debug/install/opt/yaml_cpp_vendor/lib"
  // search_paths 是ros2_foxy中的众多lib
  /*
[96]:"/home/asamu/ros2_foxy/install/orocos_kdl/lib"
[97]:"/home/asamu/ros2_foxy/install/mimick_vendor/lib"
[98]:"/home/asamu/ros2_foxy/install/message_filters/lib"
[99]:"/home/asamu/ros2_foxy/install/libyaml_vendor/lib"
[100]:"/home/asamu/ros2_foxy/install/google_benchmark_vendor/lib"
[101]:"/home/asamu/ros2_foxy/install/fastrtps/lib"
[102]:"/home/asamu/ros2_foxy/install/fastcdr/lib"
[103]:"/home/asamu/ros2_foxy/install/cyclonedds/lib"
[104]:"/home/asamu/ros2_foxy/install/console_bridge_vendor/lib"
[105]:"/home/asamu/ros2_foxy/install/ament_index_cpp/lib"
[106]:"/usr/local/lib/"
  */
  for (const auto & search_path : search_paths) {
    std::string path = search_path + "/" + filename;
    if (rcutils_is_file(path.c_str())) {
      return path;
    }
  }
  return "";
}

}  // namespace rcpputils
