#include <guik/recent_files.hpp>

#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <glk/split.hpp>
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
  if (found != recent_files.end()) {
    recent_files.erase(found);
  }

  recent_files.push_front(filename);
  while (recent_files.size() > max_history) {
    recent_files.pop_back();
  }
}

void RecentFiles::push(const std::vector<std::string>& filenames) {
  for (const auto& filename : filenames) {
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
  std::filesystem::path path(recent_files[i]);
  return path.filename().string();
}

const std::unordered_map<std::string, std::string> read_ini_file(const std::string& filename) {
  std::ifstream ifs(filename);
  if (!ifs) {
    return {};
  }

  std::unordered_map<std::string, std::string> result;

  std::string line;
  while (!ifs.eof() && std::getline(ifs, line) && !line.empty()) {
    auto found = line.find('=');
    if (found == std::string::npos) {
      continue;
    }
    result[line.substr(0, found)] = line.substr(found + 1);
  }

  return result;
}

void RecentFiles::read() {
  updated = false;
  std::string ini_path = "/tmp/tmp_recent_files.ini";

  auto ini = read_ini_file(ini_path);

  auto found = ini.find(tag);
  if (found == ini.end()) {
    return;
  }

  const auto tokens = glk::split(glk::trim(found->second), ';');
  recent_files.clear();
  recent_files.insert(recent_files.begin(), tokens.begin(), tokens.end());

  auto remove_loc = std::remove_if(recent_files.begin(), recent_files.end(), [=](const std::string& file) { return file.empty() || !std::filesystem::exists(file); });
  recent_files.erase(remove_loc, recent_files.end());
}

void RecentFiles::write() {
  if (!updated) {
    return;
  }

  std::stringstream sst;
  for (int i = 0; i < recent_files.size(); i++) {
    sst << recent_files[i] << ";";
  }

  std::string ini_path = "/tmp/tmp_recent_files.ini";

  auto ini = read_ini_file(ini_path);
  ini[tag] = sst.str();

  std::ofstream ofs(ini_path);
  for (const auto& [key, value] : ini) {
    ofs << key << "=" << value << std::endl;
  }
}

}  // namespace guik