#include <portable-file-dialogs.h>

#include <guik/recent_files.hpp>

int main(int argc, char** argv) {
  guik::RecentFiles recent_files("input_directory");

  const std::string path = pfd::select_folder("Select input directory", recent_files.most_recent()).result();
  if (!path.empty()) {
    recent_files.push(path);
  }

  return 0;
}