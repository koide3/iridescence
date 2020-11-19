#ifndef GUIK_LIGHT_VIEWER_CONTEXT_HPP
#define GUIK_LIGHT_VIEWER_CONTEXT_HPP

#include <memory>
#include <unordered_map>

#include <glk/drawble.hpp>
#include <glk/colormap.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/camera/camera_control.hpp>
#include <guik/camera/projection_control.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace guik {

class LightViewerContext {
public:
  LightViewerContext(const std::string& context_name);
  virtual ~LightViewerContext();

  void draw_ui();
  void draw_gl();

  bool init_canvas(const Eigen::Vector2i& size);

  guik::ShaderSetting& shader_setting() {
    return global_shader_setting;
  }
  const guik::ShaderSetting& shader_setting() const {
    return global_shader_setting;
  }

  void lookat(const Eigen::Vector3f& pt);
  void set_draw_xy_grid(bool draw_xy_grid);
  void set_colormap(glk::COLORMAP colormap);
  void set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect);
  void enable_info_buffer();

  void clear_drawables();
  void clear_drawables(const std::function<bool(const std::string&)>& fn);

  std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr> find_drawable(const std::string& name);
  void remove_drawable(const std::string& name);
  void update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  void clear_drawable_filters();
  void add_drawable_filter(const std::string& filter_name, const std::function<bool(const std::string&)>& filter);
  void remove_drawable_filter(const std::string& filter_name);

  const std::shared_ptr<CameraControl>& get_camera_control() const;
  const std::shared_ptr<ProjectionControl>& get_projection_control() const;
  void set_camera_control(const std::shared_ptr<CameraControl>& camera_control);
  void set_projection_control(const std::shared_ptr<ProjectionControl>& projection_control);

  Eigen::Vector2i canvas_size() const {
    return canvas->size;
  }
  Eigen::Matrix4f view_matrix() const {
    return canvas->camera_control->view_matrix();
  }
  Eigen::Matrix4f projection_matrix() const {
    return canvas->projection_control->projection_matrix();
  }

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

protected:
  std::string context_name;
  std::unique_ptr<guik::GLCanvas> canvas;
  guik::ShaderSetting global_shader_setting;

  bool draw_xy_grid;

  std::unordered_map<std::string, std::function<bool(const std::string&)>> drawable_filters;
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>> drawables;
};
}  // namespace guik

#endif