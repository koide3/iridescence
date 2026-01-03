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

/// @brief Light viewer. The main visualization class in Iridescence.
class LightViewer : public guik::Application, public guik::LightViewerContext {
public:
  /// @brief Constructor. Do not call directly. Use LightViewer::instance() instead.
  LightViewer();
  virtual ~LightViewer();

public:
  /// @brief Get the singleton instance of LightViewer. The viewer insatnce is created upon the first call.
  /// @param size        Initial canvas size. If (-1, -1), use default size.
  /// @param background  If true, the viewer window is created in the background (invisible).
  /// @param title       Window title.
  /// @return            Pointer to the LightViewer instance.
  static LightViewer* instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen");
  /// @brief Destroy the singleton instance of LightViewer.
  static void destroy();
  /// @brief Check if the LightViewer instance is running.
  static bool running();

  /// @brief Spin the viewer until a mouse click event is received.
  bool spin_until_click();
  /// @brief Spin the viewer until a toggle flag is set to true.
  bool toggle_spin_once();

  /// @brief Register a UI callback function.
  /// @param name      Name of the callback.
  /// @param callback  Callback function. If nullptr, unregister the callback.
  virtual void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0) override;

  /// @brief Invoke a task in the visualization thread.
  void invoke(const std::function<void()>& func);
  /// @brief Invoke a task after rendering in the visualization thread.
  void invoke_after_rendering(const std::function<void()>& func);
  /// @brief Invoke a labeled task only once in the life time of the viewer.
  void invoke_once(const std::string& label, const std::function<void()>& func);

  /// @brief Clear everything in the viewer (text, drawables, filters).
  virtual void clear() override;
  /// @brief Clear the text buffer.
  virtual void clear_text() override;
  /// @brief Append text to the text buffer. This method is thread-safe.
  virtual void append_text(const std::string& text) override;
  /// @brief  Set the maximum text buffer size.
  /// @param size  Maximum number of lines in the text buffer.
  void set_max_text_buffer_size(int size);

  /// @brief Clear all images.
  void clear_images();
  /// @brief Remove an image by name.
  void remove_image(const std::string& name);
  /// @brief Register or update an image.
  /// @param name   Name of the image.
  /// @param image  Image texture.
  /// @param scale  Scale factor for displaying the image. If negative, auto scale is used.
  /// @param order  Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
  void update_image(const std::string& name, const std::shared_ptr<glk::Texture>& image, double scale = -1.0, int order = -1);

  /// @brief Create or get a sub viewer context.
  /// @param context_name  Name of the sub viewer context.
  /// @param canvas_size   Canvas size of the sub viewer. If (-1, -1), use the default size.
  /// @return              Sub viewer context.
  std::shared_ptr<LightViewerContext> sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

  /// @brief Show all sub viewer windows.
  void show_sub_viewers();                                                               // Set all sub viewer windows to be opened
  /// @brief Find a sub viewer context by name.
  std::shared_ptr<LightViewerContext> find_sub_viewer(const std::string& context_name);  // Returns nullptr if sub viewer does not exist
  /// @brief Remove a sub viewer context by name.
  bool remove_sub_viewer(const std::string& context_name);                               // Returns false if sub viwewer does not exist

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
  void link_plot_axis(const std::string& plot_name, int link_id, int axis);       // axis = ImAxis_X1 or ImAxis_X2, ...
  /// @brief Link multiple axes of a plot to a link ID.
  /// @param plot_name  Name of the plot.
  /// @param link_id    Link ID.
  /// @param axes       Bitwise OR of axes to be linked (ImAxis_X1, ImAxis_X2, ImAxis_Y1, and/or ImAxis_Y2).
  void link_plot_axes(const std::string& plot_name, int link_id, int axes = -1);  // axes = (1 << ImAxis_X1) | (1 << ImAxis_X2) ...
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
  /// @brief Update a plot with new data.
  /// @param plot_name  Name of the plot.
  /// @param label      Label.
  /// @param plot       Plot data.
  void update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot);

  // Plotting update methods
  /// @brief Update a line plot.
  /// @param plot_name     Name of the plot.
  /// @param label         Label.
  /// @param ys            Y data.
  /// @param line_flags    Flags for the line plot.
  /// @param max_num_data  Maximum number of data points to be displayed.
  void update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags = 0, size_t max_num_data = 8192 * 12);
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
    size_t max_num_data = 8192 * 12);

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

  // Plotting update template methods
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
  /// @brief Update a stairs plot.
  template <typename T, int D, typename Alloc>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& data, int stairs_flags = 0);
  /// @brief Update a stairs plot.
  template <typename T, typename Func>
  void update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<T>& data, const Func& transform, int stairs_flags = 0);

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
  /// @brief  Update a 2D histogram plot.
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
  /// @brief  Update a 2D histogram plot.
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

  /// @brief Show the viewer UI.
  void show_viewer_ui();
  /// @brief Show the info window.
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
  std::unordered_set<std::string> invoke_once_called;

  std::mutex post_render_invoke_requests_mutex;
  std::deque<std::function<void()>> post_render_invoke_requests;
};

/// @brief Create or get the singleton LightViewer instance.
/// @param size         Initial canvas size. If (-1, -1), use default size.
/// @param background   If true, the viewer window is created in the background (invisible).
/// @param title        Window title.
/// @return             Pointer to the LightViewer instance.
inline LightViewer* viewer(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen") {
  return LightViewer::instance(size, background, title);
}

/// @brief Create or get the singleton LightViewer instance.
/// @param title        Window title.
/// @param size         Initial canvas size. If (-1, -1), use default size.
/// @param background   If true, the viewer window is created in the background (invisible).
/// @return             Pointer to the LightViewer instance.
inline LightViewer* viewer(const std::string& title, const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false) {
  return LightViewer::instance(size, background, title);
}

/// @brief Destroy the singleton LightViewer instance.
inline void destroy() {
  LightViewer::destroy();
}

/// @brief Check if the LightViewer instance is running.
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