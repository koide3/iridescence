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

  // Forbid copy
  Texture(const Texture& temp_obj) = delete;
  Texture& operator=(const Texture& temp_obj) = delete;

  GLuint id() const;
  Eigen::Vector2i size() const;
  void set_size(const Eigen::Vector2i& size);

  void bind() const;
  void bind(GLenum target) const;
  void unbind() const;
  void unbind(GLenum target) const;

  // They are semantically const but not logically const
  const Texture& set_filer_mode(GLenum mode) const;
  const Texture& set_clamp_mode(GLenum mode) const;

  template <typename T>
  std::vector<T> read_pixels(GLuint format = GL_RGBA, GLuint type = GL_UNSIGNED_BYTE, int num_elements = 4) const;

private:
  int width;
  int height;

  GLuint internal_format;
  GLuint format;
  GLuint type;

  GLuint filter_mode;

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