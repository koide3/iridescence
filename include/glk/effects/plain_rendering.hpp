#ifndef GLK_PLAIN_EFFECT_HPP
#define GLK_PLAIN_EFFECT_HPP

#include <glk/effects/screen_effect.hpp>

namespace glk {

class PlainRendering : public ScreenEffect {
public:
  PlainRendering();
  virtual ~PlainRendering();

  void draw(const TextureRenderer& renderer, const glk::Texture& color_texture);
  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  glk::GLSLShader plain_shader;
};

}  // namespace glk

#endif