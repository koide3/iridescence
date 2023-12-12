#ifndef GUIK_VIEWER_HPP
#define GUIK_VIEWER_HPP

#include <mutex>
#include <deque>
#include <memory>
#include <unordered_map>

#include <glk/drawable.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/imgui_application.hpp>
#include <guik/viewer/plot_setting.hpp>
#include <guik/viewer/shader_setting.hpp>
#include <guik/viewer/light_viewer_context.hpp>

namespace guik {

struct PlotData;

class LightViewer : public guik::Application, public guik::LightViewerContext {
public:
  LightViewer();
  virtual ~LightViewer();

  static LightViewer* instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen");
  static void destroy();

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
  void update_image(const std::string& name, const std::shared_ptr<glk::Texture>& image, double scale = -1.0, int order = -1);

  void clear_plots();
  void remove_plot(const std::string& plot_name, const std::string& label = "");
  void setup_plot(const std::string& plot_name, int width, int height, int plot_flags = 0, int x_flags = 0, int y_flags = 0, int order = -1);
  void update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot);
  void update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags = 0, size_t max_num_data = 2048);
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    int line_flags = 0,
    size_t max_num_data = 2048);
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int scatter_flags = 0);
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags = 0);
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int stairs_flags = 0);
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags = 0);

  std::shared_ptr<LightViewerContext> sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

  std::vector<unsigned char> read_color_buffer();
  std::vector<float> read_depth_buffer(bool real_scale = true);

  void show_viewer_ui();
  void show_info_window();

private:
  class ViewerUI;
  class InfoWindow;

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version, bool background, const std::string& title) override;
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override;

  virtual void draw_ui() override;
  virtual void draw_gl() override;

private:
  static std::unique_ptr<LightViewer> inst;

  std::unique_ptr<ViewerUI> viewer_ui;
  std::unique_ptr<InfoWindow> info_window;

  std::mutex texts_mutex;
  int max_texts_size;
  std::deque<std::string> texts;
  std::unordered_map<std::string, std::function<void()>> ui_callbacks;

  std::unordered_map<std::string, std::tuple<double, std::shared_ptr<glk::Texture>, int>> images;
  std::vector<std::shared_ptr<glk::Texture>> images_in_rendering;

  std::unordered_map<std::string, PlotSetting> plot_settings;
  std::unordered_map<std::string, std::vector<std::shared_ptr<const PlotData>>> plot_data;

  std::unordered_map<std::string, std::shared_ptr<LightViewerContext>> sub_contexts;

  std::mutex invoke_requests_mutex;
  std::deque<std::function<void()>> invoke_requests;

  std::mutex post_render_invoke_requests_mutex;
  std::deque<std::function<void()>> post_render_invoke_requests;
};

inline LightViewer* viewer(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen") {
  return LightViewer::instance(size, background, title);
}

inline LightViewer* viewer(const std::string& title, const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false) {
  return LightViewer::instance(size, background, title);
}

inline void destroy() {
  LightViewer::destroy();
}

}  // namespace guik

#endif