#include <glk/frame_buffer.hpp>

#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/texture.hpp>
#include <iostream>

namespace glk {

FrameBuffer::FrameBuffer(const Eigen::Vector2i& size, int num_color_buffers, bool use_depth) : width(size[0]), height(size[1]) {
  glGenFramebuffers(1, &frame_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  GLenum attachments[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4};
  for(int i = 0; i < num_color_buffers; i++) {
    color_attachments.push_back(attachments[i]);
    color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE));
    glFramebufferTexture2D(GL_FRAMEBUFFER, attachments[i], GL_TEXTURE_2D, color_buffers[i]->id(), 0);
  }

  if(use_depth) {
    depth_buffer = std::make_shared<Texture>(size, GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);
  }

  if(num_color_buffers) {
    glDrawBuffers(num_color_buffers, color_attachments.data());
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

FrameBuffer::~FrameBuffer() {
  glDeleteFramebuffers(1, &frame_buffer);
}

void FrameBuffer::bind() {
  glGetIntegerv(GL_VIEWPORT, viewport);
  glViewport(0, 0, width, height);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
  glDrawBuffers(color_attachments.size(), color_attachments.data());
}

void FrameBuffer::unbind() const {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

int FrameBuffer::num_color_buffers() const {
  return color_buffers.size();
}

void FrameBuffer::add_color_buffer(int layout, GLuint internal_format, GLuint format, GLuint type) {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  while(color_attachments.size() <= layout) {
    color_attachments.push_back(GL_NONE);
  }

  GLenum attachment = GL_COLOR_ATTACHMENT0 + layout;
  color_attachments[layout] = attachment;
  color_buffers.push_back(std::make_shared<Texture>(Eigen::Vector2i(width, height), internal_format, format, type));
  glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, color_buffers.back()->id(), 0);

  glDrawBuffers(color_attachments.size(), color_attachments.data());
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace glk
