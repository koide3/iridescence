#include <glk/path.hpp>

#include <iostream>
#include <filesystem>
// #include <ros/package.h>

#include <glk/console_colors.hpp>

namespace glk {

namespace {
std::string data_path;
}

void set_data_path(const std::string& path) {
  data_path = path;
}

std::string find_file(const std::string& hint, const std::string& path) {
  try {
    std::filesystem::recursive_directory_iterator itr(hint);
    std::filesystem::recursive_directory_iterator end;

    for (; itr != end; itr++) {
      if (itr->path().string().find(path) != std::string::npos) {
        std::string found = itr->path().string();
        return found.substr(0, found.size() - path.size() - 1);
      }
    }
  } catch (std::exception&) {
    return "";
  }

  return "";
}

std::string find_file(const std::vector<std::string>& hints, const std::string& path) {
  std::string found;
  for (const auto& hint : hints) {
    found = find_file(hint, path);
    if (!found.empty()) {
      break;
    }
  }

  return found;
}

std::string get_data_path() {
  if (data_path.empty()) {
    std::vector<std::string> hints;
    hints.push_back("./data");
    hints.push_back("../data");
#ifdef DATA_INSTALL_PATH
    hints.push_back(DATA_INSTALL_PATH);
#endif
    hints.push_back("/usr/share/iridescence");
    hints.push_back("/usr/local/share/iridescence");
#ifdef DATA_PATH_GUESS
    hints.push_back(DATA_PATH_GUESS);
#endif

    data_path = find_file(hints, "shader/rainbow.vert");
    if (data_path.empty()) {
      std::cerr << console::bold_red << "error: data directory not found!!" << console::reset << std::endl;
      data_path = "./data";
    }

    // std::cout << "data_path:" << data_path << std::endl;
  }

  return data_path;
}

}  // namespace glk