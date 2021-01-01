#ifndef GLK_NAIVE_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP
#define GLK_NAIVE_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP

#include <glk/effects/screen_effect.hpp>

namespace glk {

class NaiveScreenSpaceAmbientOcclusion : public ScreenEffect {
public:
  NaiveScreenSpaceAmbientOcclusion();
  virtual ~NaiveScreenSpaceAmbientOcclusion() override;

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  glk::GLSLShader ssao_shader;
};

}  // namespace glk

#endif