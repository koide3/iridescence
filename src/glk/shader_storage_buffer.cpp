#include <glk/shader_storage_buffer.hpp>

#include <cstdint>
#include <Eigen/Core>
#include <glk/console_colors.hpp>
#include <glk/async_buffer_copy.hpp>

namespace glk {

ShaderStorageBuffer::ShaderStorageBuffer(size_t size, const void* data, GLenum usage) {
  this->buffer_size = size;

  glGenBuffers(1, &ssbo);
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
  glBufferData(GL_SHADER_STORAGE_BUFFER, size, nullptr, usage);

  if (data) {
    write_buffer_async(GL_SHADER_STORAGE_BUFFER, size, data);
  }

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
  GLbitfield flag = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT;
  void* dst = glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, buffer_size, flag);

  if (data) {
    std::memcpy(dst, data, buffer_size);
  } else {
    std::memset(dst, 0, buffer_size);
  }

  glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

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

template <>
void ShaderStorageBuffer::clear_data(const std::int8_t& data) {
  glClearNamedBufferData(ssbo, GL_R8, GL_RED, GL_UNSIGNED_BYTE, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const std::int32_t& data) {
  glClearNamedBufferData(ssbo, GL_R32I, GL_RED, GL_INT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const std::uint32_t& data) {
  glClearNamedBufferData(ssbo, GL_R32UI, GL_RED, GL_UNSIGNED_INT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector2i& data) {
  glClearNamedBufferData(ssbo, GL_RG32I, GL_RG, GL_INT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector3i& data) {
  glClearNamedBufferData(ssbo, GL_RGB32I, GL_RGB, GL_INT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector4i& data) {
  glClearNamedBufferData(ssbo, GL_RGBA32I, GL_RGBA, GL_INT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const float& data) {
  glClearNamedBufferData(ssbo, GL_R32F, GL_RED, GL_FLOAT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector2f& data) {
  glClearNamedBufferData(ssbo, GL_RG32F, GL_RG, GL_FLOAT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector3f& data) {
  glClearNamedBufferData(ssbo, GL_RGB32F, GL_RGB, GL_FLOAT, &data);
}

template <>
void ShaderStorageBuffer::clear_data(const Eigen::Vector4f& data) {
  glClearNamedBufferData(ssbo, GL_RGBA32F, GL_RGBA, GL_FLOAT, &data);
}

void ShaderStorageBuffer::copy_to(ShaderStorageBuffer& dst, size_t size) const {
  if (size == 0) {
    size = buffer_size;
  }

  if (dst.buffer_size < size) {
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
