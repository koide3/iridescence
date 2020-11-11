#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <imgui.h>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

#include <guik/camera/camera_control.hpp>
#include <guik/camera/projection_control.hpp>

namespace guik {

class GLCanvas {
public:
  GLCanvas(const Eigen::Vector2i& size, const std::string& shader_name = "rainbow");

  bool ready() const;
  bool load_shader(const std::string& shader_name);

  void set_effect(const std::shared_ptr<glk::ScreenEffect>& effect);
  void set_size(const Eigen::Vector2i& size);
  void mouse_control();

  void bind();
  void unbind();

  void render_to_screen(int color_buffer_id = 0);

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  void draw_ui();

public:
  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;

  std::shared_ptr<guik::CameraControl> camera_control;
  std::shared_ptr<guik::ProjectionControl> projection_control;
};

}  // namespace guik

#endif