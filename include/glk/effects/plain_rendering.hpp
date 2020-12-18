#ifndef GLK_PLAIN_EFFECT_HPP
#define GLK_PLAIN_EFFECT_HPP

#include <glk/path.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class PlainRendering : public ScreenEffect {
public:
  PlainRendering() {
    if(!plain_shader.init(get_data_path() + "/shader/texture")) {
      return;
    }
  }
  virtual ~PlainRendering() {}

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override {
    plain_shader.use();

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_texture.id());

    renderer.draw_plain(plain_shader);

    glDisable(GL_TEXTURE_2D);
  }

private:
  glk::GLSLShader plain_shader;
};

}  // namespace glk

#endif