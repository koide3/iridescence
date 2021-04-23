#include <guik/recent_files.hpp>

#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <glk/console_colors.hpp>

namespace guik {

using namespace glk::console;

RecentFiles::RecentFiles() : tag("general"), max_history(5) {
  read();
}

RecentFiles::RecentFiles(const std::string& tag, int max_history) : tag(tag), max_history(max_history) {
  read();
}

RecentFiles::~RecentFiles() {
  write();
}

void RecentFiles::clear() {
  updated = true;
  recent_files.clear();
}

void RecentFiles::push(const std::string& filename) {
  updated = true;

  auto found = std::find(recent_files.begin(), recent_files.end(), filename);
  if(found != recent_files.end()) {
    recent_files.erase(found);
  }

  recent_files.push_front(filename);
  while(recent_files.size() > max_history) {
    recent_files.pop_back();
  }
}

void RecentFiles::push(const std::vector<std::string>& filenames) {
  for(const auto& filename : filenames) {
    push(filename);
  }
}

bool RecentFiles::empty() const {
  return recent_files.empty();
}

size_t RecentFiles::size() const {
  return recent_files.size();
}

std::string RecentFiles::most_recent() const {
  return recent_files.empty() ? "" : recent_files.front();
}

std::string RecentFiles::fullpath(size_t i) const {
  return recent_files[i];
}

std::string RecentFiles::filename(size_t i) const {
  boost::filesystem::path path(recent_files[i]);
  return path.filename().string();
}

void RecentFiles::read() {
  updated = false;
  std::string ini_path = "/tmp/tmp_recent_files.ini";

  std::cout << tag << std::endl;
  try {
    using namespace boost::property_tree;
    ptree pt;
    read_ini(ini_path, pt);

    boost::optional<std::string> files = pt.get_optional<std::string>(tag);
    if(!files) {
      return;
    }

    boost::algorithm::split(recent_files, *files, boost::is_any_of(";"));

    auto remove_loc = std::remove_if(recent_files.begin(), recent_files.end(), [=](const std::string& file) { return file.empty() || !boost::filesystem::exists(file); });
    recent_files.erase(remove_loc, recent_files.end());
  } catch(std::exception e) {
    // std::cerr << "warning:" << ini_path << " not found" << std::endl;
  }
}

void RecentFiles::write() {
  if(!updated) {
    return;
  }

  std::stringstream sst;
  for(int i = 0; i < recent_files.size(); i++) {
    sst << recent_files[i] << ";";
  }

  std::string ini_path = "/tmp/tmp_recent_files.ini";

  using namespace boost::property_tree;
  ptree pt;
  try {
    read_ini(ini_path, pt);
  } catch(std::exception e) {
    // std::cerr << "warning:" << ini_path << " not found" << std::endl;
  }

  try {
    pt.put(tag, sst.str());
    write_ini(ini_path, pt);
  } catch(std::exception e) {
    std::cerr << bold_yellow << "warning: failed to save recent files to " << ini_path << reset << std::endl;
  }
}

}  // namespace guik