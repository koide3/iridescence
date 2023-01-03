#include <glk/io/image_io.hpp>

#include <iostream>
#include <glk/io/png_io.hpp>
#include <glk/io/jpeg_io.hpp>
#include <glk/console_colors.hpp>

namespace glk {

bool load_image(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes) {
  const auto ends_with = [](const std::string& s, const std::string& suffix) {
    return s.size() < suffix.size() ? false : std::equal(std::rbegin(suffix), std::rend(suffix), std::rbegin(s));
  };

  if (ends_with(filename, ".png") || ends_with(filename, ".PNG")) {
    if (!glk::load_png(filename, width, height, bytes)) {
      std::cerr << console::yellow << "warning: failed to load " << filename << console::reset << std::endl;
      return false;
    }
  } else if (ends_with(filename, ".jpg") || ends_with(filename, ".JPG")) {
    if (!glk::load_jpeg(filename, width, height, bytes)) {
      std::cerr << console::yellow << "warning: failed to load " << filename << console::reset << std::endl;
      return false;
    }
  } else {
    std::cerr << console::yellow << "warning: unsupported image type " << filename << console::reset << std::endl;
    return false;
  }

  return true;
}

}  // namespace glk
