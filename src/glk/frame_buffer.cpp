#include <glk/frame_buffer.hpp>

#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/texture.hpp>

namespace glk {

FrameBuffer::FrameBuffer(const Eigen::Vector2i& size, int num_color_buffers) : width(size[0]), height(size[1]) {
  for (int i = 0; i < num_color_buffers; i++) {
    color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE));
    // color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA32I,
    // GL_RGBA_INTEGER, GL_INT));
  }
  depth_buffer = std::make_shared<Texture>(size, GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);

  glGenFramebuffers(1, &frame_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  GLenum color_attachments[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  for (int i = 0; i < num_color_buffers; i++) {
    glFramebufferTexture2D(GL_FRAMEBUFFER, color_attachments[i], GL_TEXTURE_2D, color_buffers[i]->id(), 0);
  }
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);

  glDrawBuffers(num_color_buffers, color_attachments);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

FrameBuffer::~FrameBuffer() { glDeleteFramebuffers(1, &frame_buffer); }

void FrameBuffer::bind() {
  glGetIntegerv(GL_VIEWPORT, viewport);
  glViewport(0, 0, width, height);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
}

void FrameBuffer::unbind() const {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void FrameBuffer::add_color_buffer(GLuint internal_format, GLuint format, GLuint type) {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  color_buffers.push_back(std::make_shared<Texture>(color_buffers.front()->size(), internal_format, format, type));
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + color_buffers.size() - 1, GL_TEXTURE_2D, color_buffers.back()->id(), 0);

  std::vector<GLuint> color_attachments(color_buffers.size());
  for(int i = 0; i < color_buffers.size(); i++) {
    color_attachments[i] = GL_COLOR_ATTACHMENT0 + i;
  }
  glDrawBuffers(color_buffers.size(), color_attachments.data());

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace glk
