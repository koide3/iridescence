#ifndef GUIK_LIGHT_VIEWER_CONTEXT_HPP
#define GUIK_LIGHT_VIEWER_CONTEXT_HPP

#include <memory>
#include <unordered_map>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace guik {

class LightViewerContext {
public:
  LightViewerContext(const std::string& context_name);
  virtual ~LightViewerContext();

  void draw_ui();
  void draw_gl();

  bool init_canvas(const Eigen::Vector2i& size);

  void lookat(const Eigen::Vector3f& pt);
  void set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect);

  void clear();
  void clear_drawables();
  void clear_drawables(const std::function<bool(const std::string&)>& fn);

  std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr> find_drawable(const std::string& name);
  void remove_drawable(const std::string& name);
  void update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  Eigen::Vector2i canvas_size() const { return canvas->size; }
  Eigen::Matrix4f view_matrix() const { return canvas->camera_control->view_matrix(); }
  Eigen::Matrix4f projection_matrix() const { return canvas->projection_control->projection_matrix(); }

protected:
  std::string context_name;
  std::unique_ptr<guik::GLCanvas> canvas;
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>> drawables;
};

}

#endif