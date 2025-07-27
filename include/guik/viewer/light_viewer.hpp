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
struct PlotStyle;
using PlotDataConstPtr = std::shared_ptr<const PlotData>;
using PlotStyleConstPtr = std::shared_ptr<const PlotStyle>;

class LightViewer : public guik::Application, public guik::LightViewerContext {
public:
  LightViewer();
  virtual ~LightViewer();

  static LightViewer* instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen");
  static void destroy();
  static bool running();

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

  std::shared_ptr<LightViewerContext> sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

  void show_sub_viewers();                                                               // Set all sub viewer windows to be opened
  std::shared_ptr<LightViewerContext> find_sub_viewer(const std::string& context_name);  // Returns nullptr if sub viewer does not exist
  bool remove_sub_viewer(const std::string& context_name);                               // Returns false if sub viwewer does not exist

  // Plotting methods
  void clear_plots(bool clear_settings = true);
  void remove_plot(const std::string& plot_name, const std::string& label = "");
  void setup_plot(const std::string& plot_name, int width, int height, int plot_flags = 0, int x_flags = 0, int y_flags = 0, int order = -1);
  void link_plot_axes(const std::string& plot_name, int link_id, int axis);
  void setup_legend(const std::string& plot_name, int loc, int flags = 0);
  void fit_plot(const std::string& plot_name);
  void fit_all_plots();
  void setup_plot_group_order(const std::string& group_name, int order);
  void update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot);

  // Plotting update methods
  void update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags = 0, size_t max_num_data = 8192 * 12);
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    int line_flags = 0,
    size_t max_num_data = 8192 * 12);
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int scatter_flags = 0);
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags = 0);
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int stairs_flags = 0);
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags = 0);
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    int bins = -2,
    const Eigen::Vector2d& range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    int x_bins = -2,
    int y_bins = -2,
    const Eigen::Vector2d& x_range = Eigen::Vector2d(0.0, 0.0),
    const Eigen::Vector2d& y_range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);

  // Plotting update template methods
  template <typename T>
  auto update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int line_flags = 0, size_t max_num_data = 8192 * 12)
    -> std::enable_if_t<std::is_arithmetic_v<T>, void>;
  template <typename T1, typename T2>
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T1>& xs,
    const std::vector<T2>& ys,
    int line_flags = 0,
    size_t max_num_data = 8192 * 12);
  template <typename T, int D, typename Alloc>
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
    int line_flags = 0,
    size_t max_num_data = 8192 * 12);
  template <typename T, typename Func>
  auto
  update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& data, const Func& transform, int line_flags = 0, size_t max_num_data = 8192 * 12)
    -> std::enable_if_t<!std::is_arithmetic_v<decltype(transform(data[0]))>, void> {
    std::vector<double> xs(data.size());
    std::vector<double> ys(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      const auto pt = transform(data[i]);
      xs[i] = pt[0];
      ys[i] = pt[1];
    }
    update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
  }
  template <typename T, typename Func>
  auto
  update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& data, const Func& transform, int line_flags = 0, size_t max_num_data = 8192 * 12)
    -> std::enable_if_t<std::is_arithmetic_v<decltype(transform(data[0]))>, void> {
    std::vector<double> xs(data.size());
    std::vector<double> ys(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      const auto pt = transform(data[i]);
      xs[i] = i;
      ys[i] = pt;
    }
    update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
  }

  template <typename T>
  auto update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int scatter_flags = 0)
    -> std::enable_if_t<std::is_arithmetic_v<T>, void>;
  template <typename T1, typename T2>
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int scatter_flags = 0);
  template <typename T, int D, typename Alloc>
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int scatter_flags = 0);
  template <typename T, typename Func>
  auto update_plot_scatter(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T>& data,
    const Func& transform,
    int scatter_flags = 0,
    size_t max_num_data = 8192 * 12) -> std::enable_if_t<!std::is_arithmetic_v<decltype(transform(data[0]))>, void> {
    std::vector<double> xs(data.size());
    std::vector<double> ys(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      const auto pt = transform(data[i]);
      xs[i] = pt[0];
      ys[i] = pt[1];
    }
    update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
  }
  template <typename T, typename Func>
  auto update_plot_scatter(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T>& data,
    const Func& transform,
    int scatter_flags = 0,
    size_t max_num_data = 8192 * 12) -> std::enable_if_t<std::is_arithmetic_v<decltype(transform(data[0]))>, void> {
    std::vector<double> xs(data.size());
    std::vector<double> ys(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      const auto pt = transform(data[i]);
      xs[i] = i;
      ys[i] = pt;
    }
    update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
  }

  template <typename T>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int stairs_flags = 0);
  template <typename T1, typename T2>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int stairs_flags = 0);
  template <typename T, int D, typename Alloc>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int stairs_flags = 0);
  template <typename T, typename Func>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& data, const Func& transform, int stairs_flags = 0);

  template <typename T>
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T>& xs,
    int bins = -2,
    const Eigen::Vector2d& range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  template <typename T1, typename T2>
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T1>& xs,
    const std::vector<T2>& ys,
    int x_bins = -2,
    int y_bins = -2,
    const Eigen::Vector2d& x_range = Eigen::Vector2d(0.0, 0.0),
    const Eigen::Vector2d& y_range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  template <typename T, int D, typename Alloc>
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
    int x_bins = -2,
    int y_bins = -2,
    const Eigen::Vector2d& x_range = Eigen::Vector2d(0.0, 0.0),
    const Eigen::Vector2d& y_range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  template <typename T, typename Func>
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T>& data,
    const Func& transform,
    int x_bins = -2,
    int y_bins = -2,
    const Eigen::Vector2d& x_range = Eigen::Vector2d(0.0, 0.0),
    const Eigen::Vector2d& y_range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);

  // Plotting style methods
  void set_plot_style(const std::string& plot_name, const std::string& label, const PlotStyleConstPtr& style);
  void set_line_style(const std::string& plot_name, const std::string& label, const Eigen::Vector4f& color = Eigen::Vector4f(0, 0, 0, -1), float weight = -1);
  void set_scatter_style(
    const std::string& plot_name,
    const std::string& label,
    int marker = 0,
    float size = -1,
    const Eigen::Vector4f& fill = Eigen::Vector4f(0, 0, 0, -1),
    float weight = -1,
    const Eigen::Vector4f& outline = Eigen::Vector4f(0, 0, 0, -1));

  // Buffer read methods
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

  bool toggle_spin_stop_flag;

  std::unordered_map<std::string, std::function<void()>> ui_callbacks;

  std::unordered_map<std::string, std::tuple<double, std::shared_ptr<glk::Texture>, int>> images;
  std::vector<std::shared_ptr<glk::Texture>> images_in_rendering;

  std::unordered_map<int, Eigen::Matrix<double, 6, 2>> plot_linked_axis_limits;
  std::unordered_map<std::string, PlotSetting> plot_settings;
  std::unordered_map<std::string, int> plot_group_orders;
  std::unordered_map<std::string, std::vector<std::pair<PlotStyleConstPtr, PlotDataConstPtr>>> plot_data;

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

inline bool running() {
  return LightViewer::running();
}

// Template methods

template <typename T>
auto LightViewer::update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int line_flags, size_t max_num_data)
  -> std::enable_if_t<std::is_arithmetic_v<T>, void> {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_line(plot_name, label, ys_, line_flags, max_num_data);
}

template <typename T1, typename T2>
void LightViewer::update_plot_line(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T1>& xs,
  const std::vector<T2>& ys,
  int line_flags,
  size_t max_num_data) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_line(plot_name, label, xs_, ys_, line_flags, max_num_data);
}

template <typename T, int D, typename Alloc>
void LightViewer::update_plot_line(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
  int line_flags,
  size_t max_num_data) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  std::transform(data.begin(), data.end(), xs.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
}

template <typename T>
auto LightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int scatter_flags)
  -> std::enable_if_t<std::is_arithmetic_v<T>, void> {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_scatter(plot_name, label, ys_, scatter_flags);
}

template <typename T1, typename T2>
void LightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int scatter_flags) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_scatter(plot_name, label, xs_, ys_, scatter_flags);
}

template <typename T, int D, typename Alloc>
void LightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int scatter_flags) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  std::transform(data.begin(), data.end(), xs.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
}

template <typename T>
void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int stairs_flags) {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_stairs(plot_name, label, ys_, stairs_flags);
}

template <typename T1, typename T2>
void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int stairs_flags) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_stairs(plot_name, label, xs_, ys_, stairs_flags);
}

template <typename T, int D, typename Alloc>
void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int stairs_flags) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  std::transform(data.begin(), data.end(), xs.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_stairs(plot_name, label, xs, ys, stairs_flags);
}

template <typename T, typename Func>
void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& data, const Func& transform, int stairs_flags) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  for (size_t i = 0; i < data.size(); i++) {
    const auto pt = transform(data[i], xs[i], ys[i]);
    xs[i] = pt[0];
    ys[i] = pt[1];
  }
  update_plot_stairs(plot_name, label, xs, ys, stairs_flags);
}

template <typename T>
void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T>& xs,
  int bins,
  const Eigen::Vector2d& range,
  int histogram_flags) {
  std::vector<double> xs_(xs.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  update_plot_histogram(plot_name, label, xs_, bins, range, histogram_flags);
}

template <typename T1, typename T2>
void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T1>& xs,
  const std::vector<T2>& ys,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_histogram(plot_name, label, xs_, ys_, x_bins, y_bins, x_range, y_range, histogram_flags);
}

template <typename T, int D, typename Alloc>
void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  std::transform(data.begin(), data.end(), xs.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_histogram(plot_name, label, xs, ys, x_bins, y_bins, x_range, y_range, histogram_flags);
}

template <typename T, typename Func>
void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T>& data,
  const Func& transform,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  std::vector<double> xs(data.size());
  std::vector<double> ys(data.size());
  for (size_t i = 0; i < data.size(); i++) {
    const auto pt = transform(data[i], xs[i], ys[i]);
    xs[i] = pt[0];
    ys[i] = pt[1];
  }
  update_plot_histogram(plot_name, label, xs, ys, x_bins, y_bins, x_range, y_range, histogram_flags);
}

}  // namespace guik

#endif