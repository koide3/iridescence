#ifndef GLK_TEXTURE_RENDERER_HPP
#define GLK_TEXTURE_RENDERER_HPP

#include <array>
#include <unordered_map>

#include <GL/gl3w.h>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer_input.hpp>

namespace glk {

class PlainRendering;

/**
 * @brief TextureRenderer
 *
 */
class TextureRenderer {
public:
  TextureRenderer();
  ~TextureRenderer();

  void draw(const glk::Texture& color_texture);
  void draw_plain(glk::GLSLShader& shader) const;

private:
  GLuint vao;
  GLuint vbo;

  std::shared_ptr<glk::PlainRendering> plain_effect;
};

}  // namespace glk

#endif