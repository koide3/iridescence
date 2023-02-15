#ifndef GLK_SHADER_STORAGE_BUFFER_HPP
#define GLK_SHADER_STORAGE_BUFFER_HPP

#include <vector>
#include <iostream>
#include <GL/gl3w.h>

namespace glk {

class ShaderStorageBuffer {
public:
  ShaderStorageBuffer(size_t size, const void* data = nullptr, GLenum usage = GL_DYNAMIC_COPY);
  ~ShaderStorageBuffer();

  GLuint id() const;
  size_t size() const;

  void set_data(size_t buffer_size, const void* data);
  void get_data(size_t buffer_size, void* data) const;

  void bind(int index = 0);
  void unbind(int index = 0);

private:
  size_t buffer_size;
  GLuint ssbo;
};

}  // namespace glk

#endif