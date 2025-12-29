#include <glk/frame_buffer.hpp>

#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/texture.hpp>

namespace glk {

FrameBuffer::FrameBuffer(const Eigen::Vector2i& size, int num_color_buffers, bool use_depth) : width(size[0]), height(size[1]) {
  glGenFramebuffers(1, &frame_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  GLenum attachments[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4};
  for (int i = 0; i < num_color_buffers; i++) {
    color_attachments.push_back(attachments[i]);
    color_buffer_layouts.push_back(i);
    color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE));
    glFramebufferTexture2D(GL_FRAMEBUFFER, attachments[i], GL_TEXTURE_2D, color_buffers[i]->id(), 0);

    color_buffers.back()->bind();
    const GLint swizzle[] = {GL_RED, GL_GREEN, GL_BLUE, GL_ONE};
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);
    color_buffers.back()->unbind();
  }

  if (use_depth) {
    depth_buffer = std::make_shared<Texture>(size, GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);
  }

  if (num_color_buffers) {
    glDrawBuffers(num_color_buffers, color_attachments.data());
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

FrameBuffer::~FrameBuffer() {
  glDeleteFramebuffers(1, &frame_buffer);
}

void FrameBuffer::set_size(const Eigen::Vector2i& size) {
  width = size[0];
  height = size[1];

  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  for (int i = 0; i < color_buffers.size(); i++) {
    color_buffers[i]->set_size(size);
    GLenum attachment = GL_COLOR_ATTACHMENT0 + color_buffer_layouts[i];
    glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, color_buffers[i]->id(), 0);
  }
  if (depth_buffer) {
    depth_buffer->set_size(size);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);
  }
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

Eigen::Vector2i FrameBuffer::size() const {
  return Eigen::Vector2i(width, height);
}

const Texture& FrameBuffer::color() const {
  return *color_buffers.at(0);
}

const Texture& FrameBuffer::color(int i) const {
  return *color_buffers.at(i);
}

const Texture& FrameBuffer::depth() const {
  return *depth_buffer;
}

Texture& FrameBuffer::color() {
  return *color_buffers.at(0);
}

Texture& FrameBuffer::color(int i) {
  return *color_buffers.at(i);
}

Texture& FrameBuffer::depth() {
  return *depth_buffer;
}

int FrameBuffer::num_color_buffers() const {
  return color_buffers.size();
}

glk::Texture& FrameBuffer::add_color_buffer(int layout, GLuint internal_format, GLuint format, GLuint type) {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  while (color_attachments.size() <= layout) {
    color_attachments.push_back(GL_NONE);
  }

  GLenum attachment = GL_COLOR_ATTACHMENT0 + layout;
  color_attachments[layout] = attachment;
  color_buffer_layouts.push_back(layout);
  color_buffers.push_back(std::make_shared<Texture>(Eigen::Vector2i(width, height), internal_format, format, type));
  glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, color_buffers.back()->id(), 0);

  glDrawBuffers(color_attachments.size(), color_attachments.data());
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  return *color_buffers.back();
}

glk::Texture& FrameBuffer::add_depth_buffer(GLuint internal_format, GLuint format, GLuint type) {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
  depth_buffer = std::make_shared<Texture>(Eigen::Vector2i(width, height), internal_format, format, type);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  return *depth_buffer;
}

void FrameBuffer::bind_ext_depth_buffer(const glk::Texture& depth_texture) {
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture.id(), 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace glk
