#ifndef GLK_IMAGE_IO_HPP
#define GLK_IMAGE_IO_HPP

#include <vector>
#include <string>

namespace glk {

/**
 * bytes are in RGBA format
 */
bool load_image(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes);

}  // namespace glk

#endif