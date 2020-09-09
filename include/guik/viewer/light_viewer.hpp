#ifndef GUIK_VIEWER_HPP
#define GUIK_VIEWER_HPP

#include <memory>
#include <unordered_map>

#include <glk/drawble.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>
#include <guik/viewer/shader_setting.hpp>
#include <guik/viewer/light_viewer_context.hpp>

namespace guik {

class LightViewer : public guik::Application, public guik::LightViewerContext {
public:
  static LightViewer* instance() {
    if(!inst) {
      inst = new LightViewer();
      inst->init(Eigen::Vector2i(1920, 1080), "#version 130");
    }

    return inst;
  }

  void clear();
  void clear_text();
  void append_text(const std::string& text);

  bool spin_until_click();
  void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);

  std::shared_ptr<LightViewerContext> sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

  void show_info_window();

private:
  class InfoWindow;

  LightViewer();
  virtual ~LightViewer();

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version) override;
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override;

  virtual void draw_ui() override;
  virtual void draw_gl() override;

private:
  static LightViewer* inst;

  std::unique_ptr<InfoWindow> info_window;

  std::vector<std::string> texts;
  std::unordered_map<std::string, std::function<void()>> ui_callbacks;

  std::unordered_map<std::string, std::shared_ptr<LightViewerContext>> sub_contexts;
};

}  // namespace plio

#endif