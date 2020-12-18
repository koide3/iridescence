#include <glk/path.hpp>

#include <iostream>
#include <ros/package.h>

namespace glk {

namespace {
std::string data_path;
}

void set_data_path(const std::string& path) {
  data_path = path;
}

std::string get_data_path() {
  if(data_path.empty()) {
    data_path = ros::package::getPath("iridescence") + "/data";
    std::cout << "data_path:" << data_path << std::endl;
  }

  return data_path;
}

}  // namespace glk