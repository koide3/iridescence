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

  void clear_plots(bool clear_settings = true);
  void remove_plot(const std::string& plot_name, const std::string& label = "");
  void setup_plot(const std::string& plot_name, int width, int height, int plot_flags = 0, int x_flags = 0, int y_flags = 0, int order = -1);
  void fit_plot(const std::string& plot_name);
  void fit_all_plots();
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

}  // namespace guik

#endif