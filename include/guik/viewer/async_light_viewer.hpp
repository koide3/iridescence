#ifndef GUIK_ASYNC_LIGHT_VIEWER_HPP
#define GUIK_ASYNC_LIGHT_VIEWER_HPP

#include <atomic>
#include <thread>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer_context.hpp>

namespace guik {

class AsyncLightViewer : public AsyncLightViewerContext {
private:
  AsyncLightViewer(const Eigen::Vector2i& size, bool background, const std::string& title);

public:
  virtual ~AsyncLightViewer();

  /// @brief Create and run the viewer in a background thread
  static AsyncLightViewer* instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen");

  /// @brief Destroy the viewer in the background thread
  static void destroy();

  /// @brief Wait for the viewer to be closed
  static void wait();

  /// @brief Wait for a break button to be clicked
  static void wait_until_click();

  /// @brief Wait while a toggle checkbox is checked
  static void toggle_wait();

  void invoke(const std::function<void()>& func);
  void invoke_after_rendering(const std::function<void()>& func);

  void clear_images();
  void remove_image(const std::string& name);
  void update_image(const std::string& name, int width, int height, const std::vector<unsigned char>& rgba_bytes, double scale = -1.0, int order = -1);

  // Plotting methods
  void clear_plots(bool clear_settings = true);
  void remove_plot(const std::string& plot_name, const std::string& label = "");
  void setup_plot(const std::string& plot_name, int width, int height, int plot_flags = 0, int x_flags = 0, int y_flags = 0, int order = -1);
  void link_plot_axes(const std::string& plot_name, int link_id, int axis);
  void setup_legend(const std::string& plot_name, int loc, int flags = 0);
  void fit_plot(const std::string& plot_name);
  void fit_all_plots();
  void setup_plot_group_order(const std::string& group_name, int order);

  // Update plot methods
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

  // Update plot template methods
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

  // Set plot style methods
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

  // This method causes synchronization with the visualization thread.
  // Do not call this frequently.
  AsyncLightViewerContext async_sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

private:
  void ui_callback();

private:
  static std::unique_ptr<AsyncLightViewer> inst;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::atomic_bool toggle_state;
  std::atomic_bool toggle_step;
  std::atomic_bool show_toggle;
  std::atomic_uint64_t toggle_count;
};

inline AsyncLightViewer* async_viewer(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen") {
  return AsyncLightViewer::instance(size, background, title);
}

inline void async_destroy() {
  AsyncLightViewer::destroy();
}

inline void async_wait() {
  AsyncLightViewer::wait();
}

inline void async_wait_until_click() {
  AsyncLightViewer::wait_until_click();
}

inline void async_toggle_wait() {
  AsyncLightViewer::toggle_wait();
}

// Template methods

template <typename T>
auto AsyncLightViewer::update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int line_flags, size_t max_num_data)
  -> std::enable_if_t<std::is_arithmetic_v<T>, void> {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_line(plot_name, label, ys_, line_flags, max_num_data);
}

template <typename T1, typename T2>
void AsyncLightViewer::update_plot_line(
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
void AsyncLightViewer::update_plot_line(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
  int line_flags,
  size_t max_num_data) {
  std::vector<double> xs_(data.size());
  std::vector<double> ys_(data.size());
  std::transform(data.begin(), data.end(), xs_.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys_.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_line(plot_name, label, xs_, ys_, line_flags, max_num_data);
}

template <typename T>
auto AsyncLightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int scatter_flags)
  -> std::enable_if_t<std::is_arithmetic_v<T>, void> {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_scatter(plot_name, label, ys_, scatter_flags);
}

template <typename T1, typename T2>
void AsyncLightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int scatter_flags) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_scatter(plot_name, label, xs_, ys_, scatter_flags);
}

template <typename T, int D, typename Alloc>
void AsyncLightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int scatter_flags) {
  std::vector<double> xs_(data.size());
  std::vector<double> ys_(data.size());
  std::transform(data.begin(), data.end(), xs_.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[0]; });
  std::transform(data.begin(), data.end(), ys_.begin(), [](const Eigen::Matrix<T, D, 1>& v) { return v[1]; });
  update_plot_scatter(plot_name, label, xs_, ys_, scatter_flags);
}

template <typename T>
void AsyncLightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int stairs_flags) {
  std::vector<double> ys_(ys.size());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_stairs(plot_name, label, ys_, stairs_flags);
}

template <typename T1, typename T2>
void AsyncLightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int stairs_flags) {
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_stairs(plot_name, label, xs_, ys_, stairs_flags);
}

template <typename T>
void AsyncLightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T>& xs,
  int bins,
  const Eigen::Vector2d& range,
  int histogram_flags) {
  //
  std::vector<double> xs_(xs.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  update_plot_histogram(plot_name, label, xs_, bins, range, histogram_flags);
}

template <typename T1, typename T2>
void AsyncLightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<T1>& xs,
  const std::vector<T2>& ys,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  //
  std::vector<double> xs_(xs.size());
  std::vector<double> ys_(ys.size());
  std::copy(xs.begin(), xs.end(), xs_.begin());
  std::copy(ys.begin(), ys.end(), ys_.begin());
  update_plot_histogram(plot_name, label, xs_, ys_, x_bins, y_bins, x_range, y_range, histogram_flags);
}

}  // namespace guik

#endif