#ifndef GUIK_RECENT_FILES_HPP
#define GUIK_RECENT_FILES_HPP

#include <deque>
#include <string>

namespace guik {

class RecentFiles {
public:
  RecentFiles();
  RecentFiles(const std::string& tag, int max_history = 10);
  ~RecentFiles();

  void clear();
  void push(const std::string& filename);

  bool empty() const;
  size_t size() const;

  std::string most_recent() const;
  std::string fullpath(size_t i) const;
  std::string filename(size_t i) const;

private:
  void read();
  void write();

private:
  bool updated;
  std::string tag;
  int max_history;

  std::deque<std::string> recent_files;
};

}  // namespace guik

#endif