#ifndef GLK_ATOMIC_COUNTERS_HPP
#define GLK_ATOMIC_COUNTERS_HPP

#include <vector>
#include <GL/gl3w.h>

namespace glk {

class AtomicCounters {
public:
  AtomicCounters(int num_counters, GLenum usage = GL_DYNAMIC_COPY) {
    this->num_counters = num_counters;

    glGenBuffers(1, &atomic_buffer);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buffer);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint) * num_counters, nullptr, usage);
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
  }
  ~AtomicCounters() {
    glDeleteBuffers(1, &atomic_buffer);
  }

  void reset(GLuint* values = nullptr) {
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buffer);

    if(values) {
      glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint) * num_counters, values);
    } else {
      std::vector<GLuint> v(num_counters, 0);
      glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint) * num_counters, v.data());
    }

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);
  }

  GLuint get_value() const {
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buffer);

    GLuint value;
    glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint), &value);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);

    return value;
  }

  std::vector<GLuint> get_values() const {
    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomic_buffer);

    std::vector<GLuint> values(num_counters, 0);
    glGetBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint) * num_counters, values.data());

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, 0);

    return values;
  }

  void bind(GLuint index = 0) const {
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, index, atomic_buffer);
  }

  void unbind() const {
    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, 0);
  }

private:
  int num_counters;
  GLuint atomic_buffer;
};
}  // namespace glk

#endif