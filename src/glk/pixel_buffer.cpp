#include <glk/pixel_buffer.hpp>

#include <iostream>

namespace glk {

PixelBuffer::PixelBuffer(const Eigen::Vector2i& size, int bytes_per_pixel) : width(size[0]), height(size[1]), bytes_per_pixel(bytes_per_pixel) {
  glGenBuffers(1, &pixel_buffer);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffer);

  glBufferData(GL_PIXEL_PACK_BUFFER, width * height * bytes_per_pixel, 0, GL_DYNAMIC_COPY);

  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

PixelBuffer::~PixelBuffer() {
  glDeleteBuffers(1, &pixel_buffer);
}

void PixelBuffer::copy_from_texture(const glk::Texture& texture, GLuint format, GLuint type) {
  glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffer);
  glBindTexture(GL_TEXTURE_2D, texture.id());
  glGetTexImage(GL_TEXTURE_2D, 0, format, type, 0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

template <typename T>
std::vector<T> PixelBuffer::read_pixels() {
  std::vector<T> data(width * height * bytes_per_pixel / sizeof(T));

  glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffer);
  GLubyte* ptr = reinterpret_cast<GLubyte*>(glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY));

  if(ptr) {
    memcpy(data.data(), ptr, width * height * bytes_per_pixel);
    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
  } else {
    std::cerr << "warning: failed to map pbo!!" << std::endl;
  }

  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

  return data;
}

template std::vector<unsigned char> PixelBuffer::read_pixels();
template std::vector<float> PixelBuffer::read_pixels();
}