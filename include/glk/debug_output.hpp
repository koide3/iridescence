#ifndef GLK_DEBUG_OUTPUT_HPP
#define GLK_DEBUG_OUTPUT_HPP

#include <string>
#include <GL/gl3w.h>

namespace glk {

void enable_debug_output();
void disable_debug_output();

void insert_debug_message(const std::string& message, GLuint id = 0);

class DebugGroup {
public:
  DebugGroup(const std::string& message, GLuint id = 0);
  ~DebugGroup();
};

}  // namespace glk

#endif