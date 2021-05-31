#ifndef GLK_SHADER_STORAGE_BUFFER_HPP
#define GLK_SHADER_STORAGE_BUFFER_HPP

#include <vector>
#include <GL/gl3w.h>

namespace glk {

class ShaderStorageBuffer {
public:
  ShaderStorageBuffer(size_t size, void* data = nullptr, GLenum usage = GL_DYNAMIC_COPY) {
    this->size = size;

    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, size, data, usage);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }

  ~ShaderStorageBuffer() {
    glDeleteBuffers(1, &ssbo);
  }

  void reset(size_t size = 0, void* data = nullptr) {
    if(size == 0) {
      size = this->size;
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    if(data) {
      glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, size, data);
    } else {
      std::vector<char> buffer(size, 0);
      glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, size, buffer.data());
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }

  void get_values(void* data) const {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, size, data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }

  void bind(int index = 0) {
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
  }

  void unbind(int index = 0) {
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, 0);
  }

private:
  size_t size;
  GLuint ssbo;
};
}  // namespace glk

#endif