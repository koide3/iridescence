#ifndef GLK_PLAIN_EFFECT_HPP
#define GLK_PLAIN_EFFECT_HPP

#include <glk/effects/screen_effect.hpp>

namespace glk {

class PlainRendering : public ScreenEffect {
public:
  PlainRendering();
  virtual ~PlainRendering();

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override ;

private:
  glk::GLSLShader plain_shader;
};

}  // namespace glk

#endif