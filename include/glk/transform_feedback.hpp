#ifndef GLK_TRANSFORM_FEEDBACK_HPP
#define GLK_TRANSFORM_FEEDBACK_HPP

#include <iostream>
#include <GL/gl3w.h>
#include <glk/drawble.hpp>

namespace glk {

class TransformFeedback : public glk::Drawable {
public:
  TransformFeedback(size_t buffer_size);
  ~TransformFeedback();

  GLuint id() const;

  void bind();
  void unbind();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  GLuint feedback;
  GLuint tbo;
};
}  // namespace glk

#endif