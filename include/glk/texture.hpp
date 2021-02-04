#ifndef GLK_TEXTURE_HPP
#define GLK_TEXTURE_HPP

#include <vector>
#include <GL/gl3w.h>
#include <Eigen/Core>

namespace glk {

class Texture {
public:
  Texture(const Eigen::Vector2i& size, GLuint internal_format, GLuint format, GLuint type, const void* pixels = nullptr);
  ~Texture();

  GLuint id() const;
  Eigen::Vector2i size() const;
  void set_size(const Eigen::Vector2i& size);

  void bind() const;
  void bind(GLenum target) const;
  void unbind() const;
  void unbind(GLenum target) const;

  template<typename T>
  std::vector<T> read_pixels(GLuint format = GL_RGBA, GLuint type = GL_UNSIGNED_BYTE) const;

private:
  int width;
  int height;

  GLuint internal_format;
  GLuint format;
  GLuint type;

  GLuint texture;
};

class Texture1D {
public:
  Texture1D(int width, GLuint internal_format, GLuint format, GLuint type, const void* pixels = nullptr);
  ~Texture1D();

  GLuint id() const;
  int size() const;

private:
  int width;
  GLuint texture;
};
}  // namespace glk

#endif