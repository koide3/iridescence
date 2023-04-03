#ifndef GLK_ASYNC_BUFFER_COPY_HPP
#define GLK_ASYNC_BUFFER_COPY_HPP

#include <GL/gl3w.h>

namespace glk {

void write_buffer_async(GLenum target, GLsizeiptr length, const void* data);
void write_named_buffer_async(GLuint buffer, GLsizeiptr length, const void* data);

}  // namespace glk

#endif