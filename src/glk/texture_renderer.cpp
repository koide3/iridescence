#include <glk/texture_renderer.hpp>

#include <iostream>
#include <glk/path.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>

namespace glk {

TextureRenderer::TextureRenderer() {
  effect = std::make_shared<PlainRendering>();

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices = {Eigen::Vector3f(-1.0f, -1.0f, 0.0f), Eigen::Vector3f(1.0f, -1.0f, 0.0f), Eigen::Vector3f(-1.0f, 1.0f, 0.0f), Eigen::Vector3f(1.0f, 1.0f, 0.0f)};

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GL_FLOAT) * 3 * vertices.size(), vertices.data(), GL_STATIC_DRAW);

  glBindVertexArray(vao);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

TextureRenderer::~TextureRenderer() {
  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void TextureRenderer::draw(const glk::Texture& color_texture, const glk::Texture& depth_texture) {
  if(effect->pass() == ScreenEffect::ONE) {
    one_pass(color_texture, depth_texture);
  } else {
    two_pass(color_texture, depth_texture);
  }
}

void TextureRenderer::one_pass(const glk::Texture& color_texture, const glk::Texture& depth_texture) {
  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, color_texture.id());
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, depth_texture.id());
  glActiveTexture(GL_TEXTURE0);

  effect->use(color_texture, depth_texture);
  GLint position_loc = effect->shader().attrib("vert_position");

  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDisable(GL_TEXTURE_2D);
}

void TextureRenderer::two_pass(const glk::Texture& color_texture, const glk::Texture& depth_texture) {
  if(!frame_buffer || frame_buffer->color(0).size() != color_texture.size()) {
    frame_buffer.reset(new glk::FrameBuffer(color_texture.size()));
    frame_buffer->add_color_buffer(GL_R32F, GL_RED, GL_FLOAT);
  }

  frame_buffer->bind();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_TEXTURE_2D);

  glBindVertexArray(vao);
  effect->use(color_texture, depth_texture, nullptr, glk::ScreenEffect::ONE);
  glEnableVertexAttribArray(effect->shader(glk::ScreenEffect::ONE).attrib("vert_position"));
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  frame_buffer->unbind();

  effect->use(color_texture, depth_texture, frame_buffer.get(), glk::ScreenEffect::TWO);
  glEnableVertexAttribArray(effect->shader(glk::ScreenEffect::TWO).attrib("vert_position"));
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDisable(GL_TEXTURE_2D);
}

}  // namespace glk