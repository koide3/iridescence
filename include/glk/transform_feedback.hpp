#ifndef GLK_TRANSFORM_FEEDBACK_HPP
#define GLK_TRANSFORM_FEEDBACK_HPP

#include <iostream>
#include <GL/gl3w.h>
#include <glk/drawable.hpp>

namespace glk {

class TransformFeedback : public glk::Drawable {
public:
  TransformFeedback(size_t buffer_size, GLenum usage = GL_STATIC_DRAW);
  ~TransformFeedback();

  GLuint id() const;

  void bind();
  void unbind();

  void read_data(intptr_t offset, size_t size, void* data);

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  GLuint feedback;
  GLuint tbo;
};
}  // namespace glk

#endif