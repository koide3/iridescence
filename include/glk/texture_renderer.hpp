#ifndef GLK_TEXTURE_RENDERER_HPP
#define GLK_TEXTURE_RENDERER_HPP

#include <array>
#include <unordered_map>
#include <boost/any.hpp>
#include <boost/optional.hpp>

#include <GL/gl3w.h>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer_input.hpp>

namespace glk {

class ScreenEffect;

/**
 * @brief TextureRenderer
 *
 */
class TextureRenderer {
public:
  TextureRenderer();
  ~TextureRenderer();

  void set_size(const Eigen::Vector2i& size);
  void set_effect(const std::shared_ptr<glk::ScreenEffect>& effect);

  void draw(const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input=nullptr);
  void draw_plain(glk::GLSLShader& shader) const;

private:
  GLuint vao;
  GLuint vbo;

  std::shared_ptr<glk::ScreenEffect> effect;
};

}  // namespace glk

#endif