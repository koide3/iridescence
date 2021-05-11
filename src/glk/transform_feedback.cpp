#include <glk/transform_feedback.hpp>

namespace glk {

TransformFeedback::TransformFeedback(size_t buffer_size, GLenum usage) {
  glGenTransformFeedbacks(1, &feedback);

  glGenBuffers(1, &tbo);
  glBindBuffer(GL_ARRAY_BUFFER, tbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, nullptr, usage);

  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, feedback);
  glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo);
  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
}

TransformFeedback::~TransformFeedback() {
  glDeleteTransformFeedbacks(1, &feedback);
  glDeleteBuffers(1, &tbo);
}

GLuint TransformFeedback::id() const {
  return feedback;
}

void TransformFeedback::bind() {
  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, feedback);
  // glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, tbo);
}

void TransformFeedback::unbind() {
  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
}

void TransformFeedback::read_data(intptr_t offset, size_t size, void* data) {
  glBindBuffer(GL_ARRAY_BUFFER, tbo);
  glGetBufferSubData(GL_ARRAY_BUFFER, offset, size, data);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void TransformFeedback::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  glBindBuffer(GL_ARRAY_BUFFER, tbo);
  glEnableVertexAttribArray(position_loc);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);

  glDrawTransformFeedback(GL_POINTS, feedback);
}

}  // namespace glk