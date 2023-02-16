#include <glk/shader_storage_buffer.hpp>

#include <glk/console_colors.hpp>

namespace glk {

ShaderStorageBuffer::ShaderStorageBuffer(size_t size, const void* data, GLenum usage) {
  this->buffer_size = size;

  glGenBuffers(1, &ssbo);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
  glBufferData(GL_SHADER_STORAGE_BUFFER, size, data, usage);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

ShaderStorageBuffer::~ShaderStorageBuffer() {
  glDeleteBuffers(1, &ssbo);
}

GLuint ShaderStorageBuffer::id() const {
  return ssbo;
}

size_t ShaderStorageBuffer::size() const {
  return buffer_size;
}

void ShaderStorageBuffer::set_data(size_t buffer_size, const void* data) {
  if (buffer_size > this->buffer_size) {
    std::cerr << console::yellow << "warning: specified input buffer size is larger than SSBO buffer size!!" << console::reset << std::endl;
    buffer_size = this->buffer_size;
  }

  glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
  if (data) {
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, buffer_size, data);
  } else {
    std::vector<char> buffer(buffer_size, 0);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, buffer_size, buffer.data());
  }
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void ShaderStorageBuffer::get_data(size_t buffer_size, void* data) const {
  if (buffer_size > this->buffer_size) {
    std::cerr << console::yellow << "warning: specified output buffer size is larger than SSBO buffer size!!" << console::reset << std::endl;
    buffer_size = this->buffer_size;
  }

  glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
  glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, buffer_size, data);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void ShaderStorageBuffer::copy_to(ShaderStorageBuffer& dst) {
  if (dst.buffer_size < buffer_size) {
    std::cerr << console::yellow << "warning: dst buffer size is smaller than src buffer size!!" << console::reset << std::endl;
  }

  glBindBuffer(GL_COPY_READ_BUFFER, ssbo);
  glBindBuffer(GL_COPY_WRITE_BUFFER, dst.id());
  glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, buffer_size);
  glBindBuffer(GL_COPY_READ_BUFFER, 0);
  glBindBuffer(GL_COPY_WRITE_BUFFER, 0);
}

void ShaderStorageBuffer::bind(int index) {
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
}

void ShaderStorageBuffer::unbind(int index) {
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, 0);
}

}  // namespace glk
