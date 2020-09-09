#ifndef GLK_TEXTURE_RENDERER_HPP
#define GLK_TEXTURE_RENDERER_HPP

#include <GL/gl3w.h>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

/**
 * @brief TextureRenderer
 *
 */
class TextureRenderer {
public:
  TextureRenderer(const std::string& data_directory);
  ~TextureRenderer();

  void draw(const glk::Texture& color_texture, const glk::Texture& depth_texture);

  void set_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
    this->effect = effect;
  }

private:
  void one_pass(const glk::Texture& color_texture, const glk::Texture& depth_texture);
  void two_pass(const glk::Texture& color_texture, const glk::Texture& depth_texture);

private:
  GLuint vao;
  GLuint vbo;

  std::unique_ptr<glk::FrameBuffer> frame_buffer;

  std::shared_ptr<glk::ScreenEffect> effect;
};

}  // namespace glk

#endif