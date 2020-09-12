#include <glk/path.hpp>

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
    data_path = ros::package::getPath("gl_test_field") + "/data";
  }

  return data_path;
}

}