#include <guik/viewer/async_light_viewer.hpp>

#include <atomic>

namespace guik {

std::unique_ptr<AsyncLightViewer> AsyncLightViewer::inst;

AsyncLightViewer::AsyncLightViewer(const Eigen::Vector2i& size, bool background, const std::string& title) {
  std::atomic_bool initiated = false;

  toggle_count = 0;
  show_toggle = false;
  toggle_state = false;
  toggle_step = false;

  kill_switch = false;
  thread = std::thread([this, size, background, title, &initiated] {
    auto viewer = guik::viewer(size, background, title);
    context = viewer;
    context->register_ui_callback("async_viewer_ui_callback", [this] { ui_callback(); });

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

void AsyncLightViewer::ui_callback() {
  if (toggle_count || show_toggle) {
    bool toggle = toggle_state;
    ImGui::Checkbox("wait", &toggle);

    ImGui::SameLine();
    if (ImGui::Button("step")) {
      toggle_step = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("hide")) {
      show_toggle = false;
    }

    toggle_state = toggle;
  }
}

void AsyncLightViewer::wait_until_click() {
  auto clicked = std::make_shared<std::atomic_bool>(false);
  const auto callback_name = "async_wait_until_click_" + guik::anon();

  inst->register_ui_callback(callback_name, [=] { *clicked = *clicked || ImGui::Button("break"); });

  while (!*clicked) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  inst->register_ui_callback(callback_name);
}

void AsyncLightViewer::toggle_wait() {
  inst->show_toggle = true;
  inst->toggle_count++;

  inst->toggle_step = false;
  while (inst->toggle_state && !inst->toggle_step) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  inst->toggle_count--;
}

void AsyncLightViewer::invoke(const std::function<void()>& func) {
  guik::viewer()->invoke(func);
}

void AsyncLightViewer::invoke_after_rendering(const std::function<void()>& func) {
  guik::viewer()->invoke_after_rendering(func);
}

void AsyncLightViewer::invoke_once(const std::string& label, const std::function<void()>& func) {
  guik::viewer()->invoke_once(label, func);
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

void AsyncLightViewer::clear_plots(bool clear_settings) {
  guik::viewer()->invoke([=] { guik::viewer()->clear_plots(clear_settings); });
}

void AsyncLightViewer::remove_plot(const std::string& plot_name, const std::string& label) {
  guik::viewer()->invoke([=] { guik::viewer()->remove_plot(plot_name, label); });
}

void AsyncLightViewer::setup_plot(const std::string& plot_name, int width, int height, int plot_flags, int x_flags, int y_flags, int order) {
  guik::viewer()->invoke([=] { guik::viewer()->setup_plot(plot_name, width, height, plot_flags, x_flags, y_flags, order); });
}

void AsyncLightViewer::link_plot_axis(const std::string& plot_name, int link_id, int axis) {
  guik::viewer()->invoke([=] { guik::viewer()->link_plot_axis(plot_name, link_id, axis); });
}

void AsyncLightViewer::link_plot_axes(const std::string& plot_name, int link_id, int axes) {
  guik::viewer()->invoke([=] { guik::viewer()->link_plot_axes(plot_name, link_id, axes); });
}

void AsyncLightViewer::setup_legend(const std::string& plot_name, int loc, int flags) {
  guik::viewer()->invoke([=] { guik::viewer()->setup_legend(plot_name, loc, flags); });
}

void AsyncLightViewer::fit_plot(const std::string& plot_name) {
  guik::viewer()->invoke([=] { guik::viewer()->fit_plot(plot_name); });
}

void AsyncLightViewer::fit_all_plots() {
  guik::viewer()->invoke([=] { guik::viewer()->fit_all_plots(); });
}

void AsyncLightViewer::setup_plot_group_order(const std::string& group_name, int order) {
  guik::viewer()->invoke([=] { guik::viewer()->setup_plot_group_order(group_name, order); });
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

void AsyncLightViewer::set_plot_style(const std::string& plot_name, const std::string& label, const PlotStyleConstPtr& style) {
  guik::viewer()->invoke([=] { guik::viewer()->set_plot_style(plot_name, label, style); });
}

void AsyncLightViewer::set_line_style(const std::string& plot_name, const std::string& label, const Eigen::Vector4f& color, float weight) {
  guik::viewer()->invoke([=] { guik::viewer()->set_line_style(plot_name, label, color, weight); });
}

void AsyncLightViewer::set_scatter_style(
  const std::string& plot_name,
  const std::string& label,
  int marker,
  float size,
  const Eigen::Vector4f& fill,
  float weight,
  const Eigen::Vector4f& outline) {
  guik::viewer()->invoke([=] { guik::viewer()->set_scatter_style(plot_name, label, marker, size, fill, weight, outline); });
}

void AsyncLightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  int bins,
  const Eigen::Vector2d& range,
  int histogram_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_histogram(plot_name, label, xs, bins, range, histogram_flags); });
}

void AsyncLightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  guik::viewer()->invoke([=] { guik::viewer()->update_plot_histogram(plot_name, label, xs, ys, x_bins, y_bins, x_range, y_range, histogram_flags); });
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
