#ifndef GUIK_VIEWER_HPP
#define GUIK_VIEWER_HPP

#include <mutex>
#include <deque>
#include <memory>
#include <unordered_map>

#include <glk/drawable.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>
#include <guik/viewer/shader_setting.hpp>
#include <guik/viewer/light_viewer_context.hpp>

namespace guik {

class LightViewer : public guik::Application, public guik::LightViewerContext {
public:
  LightViewer();
  virtual ~LightViewer();

  static std::shared_ptr<LightViewer> instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false) {
    if (!inst) {
      Eigen::Vector2i init_size = (size.array() > 0).all() ? size : Eigen::Vector2i(1920, 1080);
      inst.reset(new LightViewer());
      inst->init(init_size, "#version 330", background);
    } else {
      if ((size.array() > 0).all() && inst->window_size() != size) {
        inst->resize(size);
      }
    }

    return inst;
  }

  static void destroy() {
    if (inst) {
      inst->clear();
      inst.reset();
    }
  }

  bool spin_until_click();
  bool toggle_spin_once();
  virtual void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0) override;

  void invoke(const std::function<void()>& func);
  void invoke_after_rendering(const std::function<void()>& func);

  virtual void clear() override;
  virtual void clear_text() override;
  virtual void append_text(const std::string& text) override;
  void set_max_text_buffer_size(int size);

  void clear_images();
  void remove_image(const std::string& name);
  void update_image(const std::string& name, const std::shared_ptr<glk::Texture>& image, double scale = -1.0);

  std::shared_ptr<LightViewerContext> sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

  std::vector<unsigned char> read_color_buffer();
  std::vector<float> read_depth_buffer(bool real_scale = true);

  void show_viewer_ui();
  void show_info_window();

private:
  class ViewerUI;
  class InfoWindow;

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version, bool background) override;
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override;

  virtual void draw_ui() override;
  virtual void draw_gl() override;

private:
  static std::shared_ptr<LightViewer> inst;

  std::unique_ptr<ViewerUI> viewer_ui;
  std::unique_ptr<InfoWindow> info_window;

  std::mutex texts_mutex;
  int max_texts_size;
  std::deque<std::string> texts;
  std::unordered_map<std::string, std::function<void()>> ui_callbacks;

  std::unordered_map<std::string, std::pair<double, std::shared_ptr<glk::Texture>>> images;
  std::vector<std::shared_ptr<glk::Texture>> images_in_rendering;

  std::unordered_map<std::string, std::shared_ptr<LightViewerContext>> sub_contexts;

  std::mutex invoke_requests_mutex;
  std::deque<std::function<void()>> invoke_requests;

  std::mutex post_render_invoke_requests_mutex;
  std::deque<std::function<void()>> post_render_invoke_requests;
};

inline std::shared_ptr<LightViewer> viewer(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false) {
  return LightViewer::instance(size, background);
}

inline void destroy() {
  LightViewer::destroy();
}

}  // namespace guik

#endif