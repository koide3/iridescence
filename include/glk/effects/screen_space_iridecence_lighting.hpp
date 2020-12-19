#ifndef GLK_SCREEN_SPACE_IRIDESCENCE_LIGHTING_HPP
#define GLK_SCREEN_SPACE_IRIDESCENCE_LIGHTING_HPP

#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_scape_attribute_estimation.hpp>

namespace glk {

class ScreenSpaceIridescenceLighting : public ScreenEffect {
public:
  ScreenSpaceIridescenceLighting(const Eigen::Vector2i& size);
  virtual ~ScreenSpaceIridescenceLighting() override;

  virtual void set_size(const Eigen::Vector2i& size) override;

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override;

private:
  std::unique_ptr<glk::ScreenSpaceAttributeEstimation> ssae;

  glk::GLSLShader lighting_shader;
};

}  // namespace glk

#endif