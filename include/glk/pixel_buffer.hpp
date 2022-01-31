#ifndef GLK_PIXEL_BUFFER_HPP
#define GLK_PIXEL_BUFFER_HPP

#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/texture.hpp>

namespace glk {

class PixelBuffer {
public:
  PixelBuffer(const Eigen::Vector2i& size, int bytes_per_pixel);
  ~PixelBuffer();

  void copy_from_texture(const glk::Texture& texture, GLuint format, GLuint type);

  template <typename T>
  std::vector<T> read_pixels();

private:
  int width;
  int height;
  int bytes_per_pixel;

  GLuint pixel_buffer;
};

}

#endif