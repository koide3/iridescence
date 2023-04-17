#ifndef GLK_PATH_HPP
#define GLK_PATH_HPP

#include <string>
#include <vector>

namespace glk {

void set_data_path(const std::string& data_path);

std::string get_data_path();

std::string find_file(const std::vector<std::string>& hints, const std::string& path);

}  // namespace glk

#endif