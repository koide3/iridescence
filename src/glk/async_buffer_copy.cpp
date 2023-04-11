#include <glk/async_buffer_copy.hpp>

#include <cstring>

namespace glk {

void write_buffer_async(GLenum target, GLsizeiptr length, const void* data) {
  // GLbitfield flag = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT;
  // void* dst = glMapBufferRange(target, 0, length, flag);
  void* dst = glMapBuffer(target, GL_WRITE_ONLY);
  std::memcpy(dst, data, length);
  glUnmapBuffer(target);
}

void write_named_buffer_async(GLuint buffer, GLsizeiptr length, const void* data) {
  // GLbitfield flag = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT;
  // void* dst = glMapNamedBufferRange(buffer, 0, length, flag);

  void* dst = glMapNamedBuffer(buffer, GL_WRITE_ONLY);
  std::memcpy(dst, data, length);
  glUnmapNamedBuffer(buffer);
}

}  // namespace glk