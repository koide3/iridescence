#ifndef GLK_SCREEN_EFFECT_HPP
#define GLK_SCREEN_EFFECT_HPP

#include <glk/texture.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>

namespace glk {

class ScreenEffect {
public:
  enum PASS_TYPE { ONE, TWO };

  ScreenEffect() {}
  virtual ~ScreenEffect() {}

  virtual PASS_TYPE pass() const {
    return ONE;
  }

  virtual void use(const glk::Texture& color_texture, const glk::Texture& depth_texture, const glk::FrameBuffer* first_pass_result = nullptr, PASS_TYPE pass = ONE) {
    shader().use();
  }

  virtual glk::GLSLShader& shader(PASS_TYPE pass = ONE) = 0;
};

}  // namespace glk

#endif