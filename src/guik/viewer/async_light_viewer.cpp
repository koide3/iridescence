#include <guik/viewer/async_light_viewer.hpp>

#include <atomic>

namespace guik {

std::unique_ptr<AsyncLightViewer> AsyncLightViewer::inst;

AsyncLightViewer::AsyncLightViewer(const Eigen::Vector2i& size, bool background, const std::string& title) {
  std::atomic_bool initiated = false;

  kill_switch = false;
  thread = std::thread([this, size, background, title, &initiated] {
    auto viewer = guik::viewer(size, background, title);
    context = viewer;

    initiated = true;

    while (!kill_switch && viewer->spin_once()) {
    }
    guik::destroy();
  });

  while (!initiated) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

AsyncLightViewer::~AsyncLightViewer() {
  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

AsyncLightViewer* AsyncLightViewer::instance(const Eigen::Vector2i& size, bool background, const std::string& title) {
  if (!inst) {
    inst.reset(new AsyncLightViewer(size, background, title));
  }
  return inst.get();
}

void AsyncLightViewer::destroy() {
  inst.reset();
}

void AsyncLightViewer::wait() {
  if (inst->thread.joinable()) {
    inst->thread.join();
  }
  inst.reset();
}

void AsyncLightViewer::invoke(const std::function<void()>& func) {
  guik::viewer()->invoke(func);
}

void AsyncLightViewer::invoke_after_rendering(const std::function<void()>& func) {
  guik::viewer()->invoke_after_rendering(func);
}

void AsyncLightViewer::clear_images() {
  guik::viewer()->invoke([] { guik::viewer()->clear_images(); });
}

void AsyncLightViewer::remove_image(const std::string& name) {
  guik::viewer()->invoke([=] { guik::viewer()->remove_image(name); });
}

void AsyncLightViewer::update_image(const std::string& name, int width, int height, const std::vector<unsigned char>& rgba_bytes, double scale, int order) {
  guik::viewer()->invoke([=] {
    auto texture = std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, rgba_bytes.data());
    guik::viewer()->update_image(name, texture, scale, order);
  });
}

void AsyncLightViewer::clear_plots() {
  guik::viewer()->invoke([] { guik::viewer()->clear_plots(); });
}

void AsyncLightViewer::remove_plot(const std::string& plot_name, const std::string& label) {
  guik::viewer()->invoke([=] { guik::viewer()->remove_plot(plot_name, label); });
}

void AsyncLightViewer::setup_plot(const std::string& plot_name, int width, int height, int plot_flags, int x_flags, int y_flags, int order) {
  guik::viewer()->invoke([=] { guik::viewer()->setup_plot(plot_name, width, height, plot_flags, x_flags, y_flags, order); });
}

void AsyncLightViewer::update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot(plot_name, label, plot); });
}

void AsyncLightViewer::update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags, size_t max_num_data) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_line(plot_name, label, ys, line_flags, max_num_data); });
}

void AsyncLightViewer::update_plot_line(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  int line_flags,
  size_t max_num_data) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data); });
}

void AsyncLightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int scatter_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_scatter(plot_name, label, ys, scatter_flags); });
}

void AsyncLightViewer::update_plot_scatter(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  int scatter_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_scatter(plot_name, label, xs, ys, scatter_flags); });
}

void AsyncLightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int stairs_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_stairs(plot_name, label, ys, stairs_flags); });
}

void AsyncLightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_stairs(plot_name, label, xs, ys, stairs_flags); });
}

AsyncLightViewerContext AsyncLightViewer::async_sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size) {
  std::shared_ptr<guik::LightViewerContext> sub_viewer;
  guik::viewer()->invoke([&] { sub_viewer = guik::viewer()->sub_viewer(context_name, canvas_size); });
  while (!sub_viewer) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  }

  return AsyncLightViewerContext(sub_viewer.get());
}

}  // namespace guik
