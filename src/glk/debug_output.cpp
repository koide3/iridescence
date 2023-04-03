#include <glk/debug_output.hpp>

#include <iostream>
#include <glk/console_colors.hpp>

namespace glk {

std::string debug_source_string(GLenum source) {
  switch (source) {
    case GL_DEBUG_SOURCE_API:
      return "GL_DEBUG_SOURCE_API";
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
      return "GL_DEBUG_SOURCE_WINDOW_SYSTEM";
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
      return "GL_DEBUG_SOURCE_SHADER_COMPILER";
    case GL_DEBUG_SOURCE_THIRD_PARTY:
      return "GL_DEBUG_SOURCE_THIRD_PARTY";
    case GL_DEBUG_SOURCE_APPLICATION:
      return "GL_DEBUG_SOURCE_APPLICATION";
    case GL_DEBUG_SOURCE_OTHER:
      return "GL_DEBUG_SOURCE_OTHER";
    default:
      return "UNKNOWN_DEBUG_SOURCE";
  }
}

std::string debug_type_string(GLenum type) {
  switch (type) {
    case GL_DEBUG_TYPE_ERROR:
      return "GL_DEBUG_TYPE_ERROR";
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
      return "GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR";
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
      return "GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR";
    case GL_DEBUG_TYPE_PORTABILITY:
      return "GL_DEBUG_TYPE_PORTABILITY";
    case GL_DEBUG_TYPE_PERFORMANCE:
      return "GL_DEBUG_TYPE_PERFORMANCE";
    case GL_DEBUG_TYPE_MARKER:
      return "GL_DEBUG_TYPE_MARKER";
    case GL_DEBUG_TYPE_PUSH_GROUP:
      return "GL_DEBUG_TYPE_PUSH_GROUP";
    case GL_DEBUG_TYPE_POP_GROUP:
      return "GL_DEBUG_TYPE_POP_GROUP";
    case GL_DEBUG_TYPE_OTHER:
      return "GL_DEBUG_TYPE_OTHER";
    default:
      return "UNKNOWN_DEBUG_TYPE";
  }
}

std::string debug_severity_string(GLenum severity) {
  switch (severity) {
    case GL_DEBUG_SEVERITY_HIGH:
      return "GL_DEBUG_SEVERITY_HIGH";
    case GL_DEBUG_SEVERITY_MEDIUM:
      return "GL_DEBUG_SEVERITY_MEDIUM";
    case GL_DEBUG_SEVERITY_LOW:
      return "GL_DEBUG_SEVERITY_LOW";
    case GL_DEBUG_SEVERITY_NOTIFICATION:
      return "GL_DEBUG_SEVERITY_NOTIFICATION";
    default:
      return "UNKNOWN_DEBUG_SEVERITY";
  }
}

void set_color(GLenum severity, std::ostream& ost) {
  using namespace glk::console;

  switch (severity) {
    case GL_DEBUG_SEVERITY_HIGH:
      ost << red;
      return;

    case GL_DEBUG_SEVERITY_MEDIUM:
      ost << yellow;
      return;

    default:
      return;
  }
}

void default_debug_callback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* user_param) {
  using namespace glk::console;

  if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) {
    return;
  }

  set_color(severity, std::cerr);
  std::cerr << "*** " << debug_severity_string(severity) << " ***" << std::endl;
  std::cerr << "severity:" << debug_severity_string(severity) << std::endl;
  std::cerr << "source:" << debug_source_string(source) << " type:" << debug_type_string(type) << " id:" << id << std::endl;
  std::cerr << bold << message << reset << std::endl;
}

void insert_debug_message(const std::string& message, GLuint id) {
  glDebugMessageInsert(GL_DEBUG_SOURCE_APPLICATION, GL_DEBUG_TYPE_MARKER, id, GL_DEBUG_SEVERITY_NOTIFICATION, message.size(), message.data());
}

void enable_debug_output() {
  glEnable(GL_DEBUG_OUTPUT);
  glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  glDebugMessageCallback(default_debug_callback, nullptr);
}

void disable_debug_output() {
  glDisable(GL_DEBUG_OUTPUT);
  glDisable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
}

DebugGroup::DebugGroup(const std::string& message, GLuint id) {
  glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, id, message.size(), message.data());
}

DebugGroup::~DebugGroup() {
  glPopDebugGroup();
}

}  // namespace glk