#ifndef GLK_TEXTURE_HPP
#define GLK_TEXTURE_HPP

#include <vector>
#include <GL/gl3w.h>
#include <Eigen/Core>

namespace glk {

/// @brief Texture class to wrap OpenGL texture object
class Texture {
public:
  /// @brief Constructor
  /// @note  Default filter mode is GL_LINEAR, and default clamp mode is GL_CLAMP_TO_EDGE.
  /// @param size              Size (width, height)
  /// @param internal_format   Internal format of the texture (e.g., GL_RGBA8)
  /// @param format            Format of the pixel data (e.g., GL_RGBA)
  /// @param type              Data type of the pixel data (e.g., GL_UNSIGNED_BYTE)
  /// @param pixels            Pointer to the pixel data. If nullptr, the texture is created but not initialized.
  Texture(const Eigen::Vector2i& size, GLuint internal_format, GLuint format, GLuint type, const void* pixels = nullptr);
  ~Texture();

  // Forbid copy
  Texture(const Texture& temp_obj) = delete;
  Texture& operator=(const Texture& temp_obj) = delete;

  /// @brief Texture ID
  GLuint id() const;
  /// @brief Texture size
  Eigen::Vector2i size() const;
  /// @brief  Change the size of the texture. This will invalidate the texture data and change the texture ID.
  void set_size(const Eigen::Vector2i& size);

  /// @brief Bind the texture
  void bind() const;
  /// @brief Bind the texture to the specified target (e.g., GL_TEXTURE0)
  void bind(GLenum target) const;
  /// @brief Unbind the texture
  void unbind() const;
  /// @brief Unbind the texture from the specified target (e.g., GL_TEXTURE0)
  void unbind(GLenum target) const;

  // They are semantically const but not logically const
  /// @brief Set the filtering mode of the texture (e.g., GL_LINEAR, GL_NEAREST)
  const Texture& set_filer_mode(GLenum mode) const;
  /// @brief  Set the clamping mode of the texture (e.g., GL_CLAMP_TO_EDGE, GL_REPEAT)
  const Texture& set_clamp_mode(GLenum mode) const;

  /// @brief Read the pixel data from the texture into a vector.
  /// @tparam T            Data type of the pixel data to read (uint8, uint16, int32, uint32, or float)
  /// @param format        Format of the pixel data to read (e.g., GL_RGBA, GL_RGB, GL_RED)
  /// @param type          Data type of the pixel data to read (e.g., GL_UNSIGNED_BYTE, GL_FLOAT)
  /// @param num_elements  Number of elements per pixel (e.g., 4 for RGBA, 3 for RGB, 1 for RED)
  /// @return              Loaded pixel data.
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