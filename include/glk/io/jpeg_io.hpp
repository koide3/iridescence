#ifndef GLK_JPEG_IO_HPP
#define GLK_JPEG_IO_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

/**
 * bytes are in RGBA format
 */
bool load_jpeg(const std::string& filename, int& width, int& height, std::vector<unsigned char>& bytes);

/**
 * bytes must be in RGBA format
 */
bool save_jpeg(const std::string& filename, int width, int height, const std::vector<unsigned char>& bytes, int quality = 100);

}  // namespace glk

#endif