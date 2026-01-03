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

  /// @brief Invoke a task in the visualization thread.
  void invoke(const std::function<void()>& func);
  /// @brief Invoke a task after rendering in the visualization thread.
  void invoke_after_rendering(const std::function<void()>& func);
  /// @brief Invoke a labeled task only once in the life time of the viewer.
  void invoke_once(const std::string& label, const std::function<void()>& func);

  /// @brief Clear all images.
  void clear_images();
  /// @brief Remove an image by name.
  void remove_image(const std::string& name);
  /// @brief Register or update an image.
  /// @param name   Name of the image.
  /// @param width  Image width.
  /// @param height Image height.
  /// @param rgba_bytes  Image data in RGBA format.
  /// @param scale  Scale factor for displaying the image. If negative, auto scale is used.
  /// @param order  Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
  void update_image(const std::string& name, int width, int height, const std::vector<unsigned char>& rgba_bytes, double scale = -1.0, int order = -1);

  // Plotting methods
  /// @brief Clear all plots.
  /// @param clear_settings  If true, clear plot settings as well.
  void clear_plots(bool clear_settings = true);
  /// @brief Remove a plot by name.
  /// @param plot_name  Name of the plot to be removed.
  /// @param label      If non-empty, remove only the plot with the specified label in the plot group.
  void remove_plot(const std::string& plot_name, const std::string& label = "");
  /// @brief Setup a plot.
  /// @param plot_name  Name of the plot.
  /// @param width      Width of the plot [pixels].
  /// @param height     Height of the plot [pixels].
  /// @param plot_flags Flags for the plot.
  /// @param x_flags    Flags for the x-axis.
  /// @param y_flags    Flags for the y-axis.
  /// @param order      Display order of the plot. Plots with smaller order values are displayed first. If -1, the plot is added to the end of the display order.
  void setup_plot(const std::string& plot_name, int width, int height, int plot_flags = 0, int x_flags = 0, int y_flags = 0, int order = -1);
  /// @brief Link an axis of a plot to a link ID.
  /// @param plot_name  Name of the plot.
  /// @param link_id    Link ID.
  /// @param axis       Axis to be linked (ImAxis_X1, ImAxis_X2, ImAxis_Y1, or ImAxis_Y2).
  void link_plot_axis(const std::string& plot_name, int link_id, int axis);
  /// @brief Link multiple axes of a plot to a link ID.
  /// @param plot_name  Name of the plot.
  /// @param link_id    Link ID.
  /// @param axes       Bitwise OR of axes to be linked (ImAxis_X1, ImAxis_X2, ImAxis_Y1, and/or ImAxis_Y2).
  void link_plot_axes(const std::string& plot_name, int link_id, int axes = -1);
  /// @brief Setup the legend for a plot.
  /// @param plot_name  Name of the plot.
  /// @param loc        Location of the legend.
  /// @param flags      Flags for the legend.
  void setup_legend(const std::string& plot_name, int loc, int flags = 0);
  /// @brief Fit the plot view to the data range.
  /// @param plot_name  Name of the plot.
  void fit_plot(const std::string& plot_name);
  /// @brief Fit all plots to their data ranges.
  void fit_all_plots();
  /// @brief Setup the plot group order.
  /// @param group_name  Name of the plot group.
  /// @param order       Display order of the plot group. Plot groups with smaller order values are displayed first.
  void setup_plot_group_order(const std::string& group_name, int order);

  // Update plot methods
  /// @brief Update a plot with new data.
  /// @param plot_name  Name of the plot.
  /// @param label      Label.
  /// @param plot       Plot data.
  void update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot);
  /// @brief Update a line plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param ys            Y data.
  /// @param line_flags    Flags for the line plot.
  /// @param max_num_data  Maximum number of data points to be displayed.
  void update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags = 0, size_t max_num_data = 2048);
  /// @brief Update a line plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param xs            X data.
  /// @param ys            Y data.
  /// @param line_flags    Flags for the line plot.
  /// @param max_num_data  Maximum number of data points to be displayed.
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    int line_flags = 0,
    size_t max_num_data = 2048);
  /// @brief Update a scatter plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param ys            Y data.
  /// @param scatter_flags Flags for the scatter plot.
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int scatter_flags = 0);
  /// @brief Update a scatter plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param xs            X data.
  /// @param ys            Y data.
  /// @param scatter_flags Flags for the scatter plot.
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags = 0);
  /// @brief Update a stairs plot.
  /// @param plot_name   Name of the plot.
  /// @param label       Label.
  /// @param ys          Y data.
  /// @param stairs_flags  Flags for the stairs plot.
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int stairs_flags = 0);
  /// @brief Update a stairs plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param xs            X data.
  /// @param ys            Y data.
  /// @param stairs_flags  Flags for the stairs plot.
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags = 0);
  /// @brief Update a histogram plot.
  /// @param plot_name      Name of the plot.
  /// @param label          Label.
  /// @param xs             Data to be histogrammed.
  /// @param bins           Number of bins.
  /// @param range          Data range of the histogram.
  /// @param histogram_flags  Flags for the histogram plot.
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<double>& xs,
    int bins = -2,
    const Eigen::Vector2d& range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  /// @brief  Update a 2D histogram plot.
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
  /// @brief Update a line plot.
  template <typename T>
  auto update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int line_flags = 0, size_t max_num_data = 8192 * 12)
    -> std::enable_if_t<std::is_arithmetic_v<T>, void>;
  /// @brief Update a line plot.
  template <typename T1, typename T2>
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T1>& xs,
    const std::vector<T2>& ys,
    int line_flags = 0,
    size_t max_num_data = 8192 * 12);
  /// @brief Update a line plot.
  template <typename T, int D, typename Alloc>
  void update_plot_line(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data,
    int line_flags = 0,
    size_t max_num_data = 8192 * 12);

  /// @brief Update a line plot.
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
  /// @brief Update a line plot.
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

  /// @brief Update a scatter plot.
  template <typename T>
  auto update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int scatter_flags = 0)
    -> std::enable_if_t<std::is_arithmetic_v<T>, void>;
  /// @brief Update a scatter plot.
  template <typename T1, typename T2>
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int scatter_flags = 0);
  /// @brief Update a scatter plot.
  template <typename T, int D, typename Alloc>
  void update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int scatter_flags = 0);
  /// @brief Update a scatter plot.
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
  /// @brief Update a scatter plot.
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

  /// @brief Update a stairs plot.
  template <typename T>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& ys, int stairs_flags = 0);
  /// @brief Update a stairs plot.
  template <typename T1, typename T2>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T1>& xs, const std::vector<T2>& ys, int stairs_flags = 0);
  /// @brief Update a histogram plot.
  template <typename T>
  void update_plot_histogram(
    const std::string& plot_name,
    const std::string& label,
    const std::vector<T>& xs,
    int bins = -2,
    const Eigen::Vector2d& range = Eigen::Vector2d(0.0, 0.0),
    int histogram_flags = 0);
  /// @brief  Update a 2D histogram plot.
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
  /// @brief Set the style for a plot.
  /// @param plot_name  Name of the plot.
  /// @param label      Label.
  /// @param style      Plot style.
  void set_plot_style(const std::string& plot_name, const std::string& label, const PlotStyleConstPtr& style);
  /// @brief  Set the line style for a plot.
  /// @param plot_name  Name of the plot.
  /// @param label      Label.
  /// @param color      Line color.
  /// @param weight     Line weight.
  void set_line_style(const std::string& plot_name, const std::string& label, const Eigen::Vector4f& color = Eigen::Vector4f(0, 0, 0, -1), float weight = -1);
  /// @brief  Set the scatter style for a plot.
  /// @param plot_name  Name of the plot.
  /// @param label      Label.
  /// @param marker     Marker style.
  /// @param size       Marker size.
  /// @param fill       Marker fill color.
  /// @param weight     Marker outline weight.
  /// @param outline    Marker outline color.
  void set_scatter_style(
    const std::string& plot_name,
    const std::string& label,
    int marker = 0,
    float size = -1,
    const Eigen::Vector4f& fill = Eigen::Vector4f(0, 0, 0, -1),
    float weight = -1,
    const Eigen::Vector4f& outline = Eigen::Vector4f(0, 0, 0, -1));

  /// @brief Create a async sub viewer context.
  /// @note  This method causes synchronization with the visualization thread. Do not call this frequently.
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