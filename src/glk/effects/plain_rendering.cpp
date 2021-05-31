#include <iostream>
#include <glk/path.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/plain_rendering.hpp>

namespace glk {

PlainRendering::PlainRendering() {
  if(!plain_shader.init(get_data_path() + "/shader/texture")) {
    return;
  }
}
PlainRendering::~PlainRendering() {}

void PlainRendering::draw(const TextureRenderer& renderer, const glk::Texture& color_texture) {
  plain_shader.use();

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, color_texture.id());

  renderer.draw_plain(plain_shader);
}

void PlainRendering::draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer) {
  if(frame_buffer) {
    frame_buffer->bind();
  }

  draw(renderer, color_texture);

  if(frame_buffer) {
    frame_buffer->unbind();
  }
}

}  // namespace glk
