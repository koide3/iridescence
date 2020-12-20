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

void PlainRendering::draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) {
  plain_shader.use();

  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, color_texture.id());

  renderer.draw_plain(plain_shader);

  glDisable(GL_TEXTURE_2D);
}

}  // namespace glk
