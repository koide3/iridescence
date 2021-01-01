#ifndef GLK_SCREEN_EFFECT_HPP
#define GLK_SCREEN_EFFECT_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glk/texture.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

namespace glk {

class TextureRenderer;

class ScreenEffect {
public:
  ScreenEffect() {}
  virtual ~ScreenEffect() {}

  virtual void set_size(const Eigen::Vector2i& size) {}
  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer = nullptr) = 0;
};

}  // namespace glk

#endif