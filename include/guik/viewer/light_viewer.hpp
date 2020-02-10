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
public:
  LightViewer();
  ~LightViewer();

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version) override;

  virtual void draw_ui() override;
  virtual void draw_gl() override;

  void clear_text();
  void append_text(const std::string& text);

  void clear();
  bool spin_until_click();
  void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);
  void update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  ShaderSetting::Ptr shader_setting(const std::string& name);

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

  std::vector<std::string> texts;
  std::unordered_map<std::string, std::function<void()>> ui_callbacks;
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>> drawables;
};

}  // namespace plio

#endif