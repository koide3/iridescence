#include <glk/io/jpeg_io.hpp>

#include <cstdio>
#include <iostream>
#include <glk/console_colors.hpp>

namespace glk {

using namespace glk::console;

bool load_jpeg(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes) {
  std::cerr << bold_red << "Failed to load PNG image because Iridescence was built without libpng!!" << reset << std::endl;
  return false;
}

bool save_jpeg(const std::string& filename, int width, int height, const std::vector<unsigned char>& bytes, int quality) {
   std::cerr << bold_red << "Failed to save PNG image because Iridescence was built without libpng!!" << reset << std::endl;
  return false;
}
}  // namespace glk