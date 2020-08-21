#ifndef GUIK_VIEWER_HPP
#define GUIK_VIEWER_HPP

#include <memory>
#include <unordered_map>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace guik {

class LightViewer : public guik::Application {
private:
  class InfoWindow;

public:
  LightViewer();
  ~LightViewer();

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version) override;

  virtual void draw_ui() override;
  virtual void draw_gl() override;

  void clear_text();
  void append_text(const std::string& text);

  void lookat(const Eigen::Vector3f& pt);
  void set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect);

  void clear();
  void clear_drawables();
  void clear_drawables(const std::function<bool(const std::string&)>& fn);

  bool spin_until_click();
  void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);

  std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr> find_drawable(const std::string& name);
  void remove_drawable(const std::string& name);
  void update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  void show_info_window();

  ShaderSetting::Ptr shader_setting(const std::string& name);

  Eigen::Vector2i canvas_size() const { return canvas->size; }
  Eigen::Matrix4f view_matrix() const { return canvas->camera_control->view_matrix(); }
  Eigen::Matrix4f projection_matrix() const { return canvas->projection_control->projection_matrix(); }

  static LightViewer* instance() {
    if(!inst) {
      inst = new LightViewer();
      inst->init(Eigen::Vector2i(1920, 1080), "#version 130");
    }

    return inst;
  }

private:
  static LightViewer* inst;

  std::unique_ptr<guik::GLCanvas> canvas;

  std::unique_ptr<InfoWindow> info_window;

  std::vector<std::string> texts;
  std::unordered_map<std::string, std::function<void()>> ui_callbacks;
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>> drawables;
};

}  // namespace plio

#endif