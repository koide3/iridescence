#include <glk/texture.hpp>

#include <iostream>
#include <glk/console_colors.hpp>

namespace glk {

Texture::Texture(const Eigen::Vector2i& size, GLuint internal_format, GLuint format, GLuint type, const void* pixels)
: width(size[0]),
  height(size[1]),
  internal_format(internal_format),
  format(format),
  type(type) {
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, internal_format, size[0], size[1], 0, format, type, pixels);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);
}

Texture::~Texture() {
  glDeleteTextures(1, &texture);
}

GLuint Texture::id() const {
  return texture;
}

Eigen::Vector2i Texture::size() const {
  return Eigen::Vector2i(width, height);
}

void Texture::set_size(const Eigen::Vector2i& size) {
  width = size[0];
  height = size[1];

  glDeleteTextures(1, &texture);

  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, internal_format, size[0], size[1], 0, format, type, 0);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::unbind() const {
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::bind() const {
  glBindTexture(GL_TEXTURE_2D, texture);
}

void Texture::bind(GLenum target) const {
  glActiveTexture(target);
  glBindTexture(GL_TEXTURE_2D, texture);
  glActiveTexture(GL_TEXTURE0);
}

void Texture::unbind(GLenum target) const {
  glActiveTexture(target);
  glBindTexture(GL_TEXTURE_2D, 0);
  glActiveTexture(GL_TEXTURE0);
}

const Texture& Texture::set_filer_mode(GLenum mode) const {
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mode);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mode);
  glBindTexture(GL_TEXTURE_2D, 0);
  return *this;
}

const Texture& Texture::set_clamp_mode(GLenum mode) const {
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, mode);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, mode);
  glBindTexture(GL_TEXTURE_2D, 0);
  return *this;
}

template <typename T>
std::vector<T> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const {
  switch (format) {
    case GL_DEPTH_COMPONENT:
    case GL_RED:
    case GL_GREEN:
    case GL_BLUE:
      if (num_elements != 1) {
        std::cerr << console::bold_yellow << "warning: num_elements should be 1 for format=" << static_cast<size_t>(format) << console::reset << std::endl;
      }
      break;
    case GL_RG:
      if (num_elements != 2) {
        std::cerr << console::bold_yellow << "warning: num_elements should be 2 for format=" << static_cast<size_t>(format) << console::reset << std::endl;
      }
      break;
    case GL_RGB:
    case GL_BGR:
      if (num_elements != 3) {
        std::cerr << console::bold_yellow << "warning: num_elements should be 3 for format=" << static_cast<size_t>(format) << console::reset << std::endl;
      }
      break;
    case GL_RGBA:
    case GL_BGRA:
      if (num_elements != 4) {
        std::cerr << console::bold_yellow << "warning: num_elements should be 4 for format=" << static_cast<size_t>(format) << console::reset << std::endl;
      }
      break;
    default:
      break;
  }

  std::vector<T> pixels(width * height * num_elements);
  glBindTexture(GL_TEXTURE_2D, texture);
  glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels.data());
  glBindTexture(GL_TEXTURE_2D, 0);
  return pixels;
}

template std::vector<unsigned char> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const;
template std::vector<int> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const;
template std::vector<unsigned int> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const;
template std::vector<uint16_t> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const;
template std::vector<float> Texture::read_pixels(GLuint format, GLuint type, int num_elements) const;

Texture1D::Texture1D(int width, GLuint internal_format, GLuint format, GLuint type, const void* pixels) : width(width) {
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_1D, texture);
  glTexImage1D(GL_TEXTURE_1D, 0, internal_format, width, 0, format, type, pixels);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_1D, 0);
}

Texture1D::~Texture1D() {
  glDeleteTextures(1, &texture);
}

GLuint Texture1D::id() const {
  return texture;
}

int Texture1D::size() const {
  return width;
}
}  // namespace glk