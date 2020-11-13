#include <glk/texture_renderer.hpp>

#include <iostream>
#include <glk/path.hpp>
#include <glk/effects/plain_rendering.hpp>

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

void TextureRenderer::set_size(const Eigen::Vector2i& size) {
  effect->set_size(size);
}

void TextureRenderer::set_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
  this->effect = effect;
}

void TextureRenderer::draw(const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) {
  effect->draw(*this, color_texture, depth_texture, input);
}

void TextureRenderer::draw_plain(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  glBindVertexArray(vao);
  glEnableVertexAttribArray(position_loc);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

}  // namespace glk