#ifndef GLK_TEXTURE_HPP
#define GLK_TEXTURE_HPP

#include <vector>
#include <GL/gl3w.h>
#include <Eigen/Core>

namespace glk {

class Texture {
public:
  Texture(const Eigen::Vector2i& size, GLuint internal_format, GLuint format, GLuint type, void* pixels = nullptr) : width(size[0]), height(size[1]) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, size[0], size[1], 0, format, type, pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  ~Texture() {
    glDeleteTextures(1, &texture);
  }

  GLuint id() const {
    return texture;
  }
  Eigen::Vector2i size() const {
    return Eigen::Vector2i(width, height);
  }

  void unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  void bind() const {
    glBindTexture(GL_TEXTURE_2D, texture);
  }

  void bind(GLenum target) const {
    glActiveTexture(target);
    glBindTexture(GL_TEXTURE_2D, texture);
    glActiveTexture(GL_TEXTURE0);
  }

  template<typename T>
  std::vector<T> read_pixels(GLuint format = GL_RGBA, GLuint type = GL_UNSIGNED_BYTE) const {
    std::vector<T> pixels(width * height * 4);
    glBindTexture(GL_TEXTURE_2D, texture);
    glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels.data());
    return pixels;
  }

private:
  int width;
  int height;
  GLuint texture;
};

class Texture1D {
public:
  Texture1D(int width, GLuint internal_format, GLuint format, GLuint type, void* pixels = nullptr) : width(width) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_1D, texture);
    glTexImage1D(GL_TEXTURE_1D, 0, internal_format, width, 0, format, type, pixels);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_1D, 0);
  }
  ~Texture1D() {
    glDeleteTextures(1, &texture);
  }

  GLuint id() const {
    return texture;
  }
  int size() const {
    return width;
  }

private:
  int width;
  GLuint texture;
};
}  // namespace glk

#endif