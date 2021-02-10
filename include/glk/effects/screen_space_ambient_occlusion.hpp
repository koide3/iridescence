#ifndef GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP
#define GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP

#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceAttributeEstimation;

class ScreenSpaceAmbientOcclusion : public ScreenEffect {
public:
  ScreenSpaceAmbientOcclusion(const Eigen::Vector2i& size = Eigen::Vector2i(1920, 1080));
  virtual ~ScreenSpaceAmbientOcclusion() override;

  virtual void set_size(const Eigen::Vector2i& size) override;

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) override;

private:
  std::unique_ptr<glk::ScreenSpaceAttributeEstimation> ssae;

  glk::GLSLShader ssao_shader;
};

}  // namespace glk

#endif