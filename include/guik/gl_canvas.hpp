#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <imgui.h>

#include <glk/colormap.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <guik/camera/camera_control.hpp>
#include <guik/camera/projection_control.hpp>

namespace guik {

class GLCanvas {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GLCanvas(const Eigen::Vector2i& size, const std::string& shader_name = "rainbow", size_t num_color_buffers = 1);

  bool ready() const;
  bool load_shader(const std::string& shader_name);

  void set_size(const Eigen::Vector2i& size);
  void set_clear_color(const Eigen::Vector4f& color);
  void set_colormap(glk::COLORMAP colormap_type);
  void set_effect(const std::shared_ptr<glk::ScreenEffect>& effect);
  const std::shared_ptr<glk::ScreenEffect>& get_effect() const;
  void set_bg_texture(const std::shared_ptr<glk::Texture>& bg_texture);
  void set_blend_func(GLenum sfactor, GLenum dfactor);
  void set_blend_depth_write(bool blend_depth_write);

  void enable_normal_buffer();
  void enable_info_buffer();
  void enable_partial_rendering(double clear_thresh = 1e-6);
  void disable_partial_rendering();
  void clear_partial_rendering();

  bool normal_buffer_enabled() const;
  bool info_buffer_enabled() const;
  bool partial_rendering_enabled() const;

  const glk::Texture& color_buffer() const;
  const glk::Texture& depth_buffer() const;
  const glk::Texture& normal_buffer() const;
  const glk::Texture& info_buffer() const;
  const glk::Texture& dynamic_flag_buffer() const;

  void mouse_control();

  void bind();
  void unbind();

  void bind_second();
  void unbind_second();

  void render_to_screen(int color_buffer_id = 0);

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  void draw_ui();

public:
  Eigen::Vector2i size;
  Eigen::Vector4f clear_color;

  GLenum alpha_blend_sfactor;
  GLenum alpha_blend_dfactor;
  bool blend_depth_write;

  int normal_buffer_id;
  int info_buffer_id;
  int dynamic_flag_buffer_id;

  bool is_partial_rendering_enabled;
  bool clear_partial_rendering_flag;
  double partial_rendering_clear_thresh;
  Eigen::Matrix4f last_projection_view_matrix;

  double keyboard_control_speed;

  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::GLSLShader> texture_shader;
  std::unique_ptr<glk::GLSLShader> partial_clear_shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::FrameBuffer> screen_effect_buffer;
  std::shared_ptr<glk::ScreenEffect> screen_effect;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;
  std::shared_ptr<glk::Texture> bg_texture;

  std::unique_ptr<glk::Texture> colormap;
  std::shared_ptr<guik::CameraControl> camera_control;
  std::shared_ptr<guik::ProjectionControl> projection_control;
};

}  // namespace guik

#endif