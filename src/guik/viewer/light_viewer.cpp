#include <guik/viewer/light_viewer.hpp>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <regex>
#include <chrono>
#include <numeric>

#include <implot.h>

#include <glk/io/png_io.hpp>
#include <glk/split.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/console_colors.hpp>
#include <guik/viewer/plot_data.hpp>
#include <guik/viewer/viewer_ui.hpp>
#include <guik/viewer/info_window.hpp>

namespace guik {

std::unique_ptr<LightViewer> LightViewer::inst;

LightViewer* LightViewer::instance(const Eigen::Vector2i& size, bool background, const std::string& title) {
  if (!inst) {
    Eigen::Vector2i init_size = (size.array() > 0).all() ? size : Eigen::Vector2i(1920, 1080);
    inst.reset(new LightViewer());
    inst->init(init_size, "#version 330", background, title);
  } else {
    if ((size.array() > 0).all() && inst->window_size() != size) {
      inst->resize(size);
    }
  }

  return inst.get();
}

void LightViewer::destroy() {
  if (inst) {
    inst->clear();
    inst.reset();
  }
}

bool LightViewer::running() {
  return inst.get() != nullptr;
}

LightViewer::LightViewer() : Application(), LightViewerContext("main"), max_texts_size(32), toggle_spin_stop_flag(false) {}

LightViewer::~LightViewer() {}

bool LightViewer::init(const Eigen::Vector2i& size, const char* glsl_version, bool background, const std::string& title) {
  Application::init(size, glsl_version, background, title);
  // GL::Renderer::setClearColor(0x474747_rgbf); // lightwave
  // GL::Renderer::setClearColor(0x3a3a3a_rgbf); // blender

  if (!LightViewerContext::init_canvas(size)) {
    close();
    return false;
  }

  return true;
}

void LightViewer::framebuffer_size_callback(const Eigen::Vector2i& size) {
  // if(!LightViewerContext::init_canvas(size)) {
  //  std::cerr << "error: failed to resize the canvas!!" << std::endl;
  //  close();
  // }
  LightViewerContext::set_size(size);
}

void LightViewer::draw_ui() {
  std::unique_lock<std::mutex> lock(invoke_requests_mutex);
  std::deque<std::function<void()>> invoke_requests;
  invoke_requests.swap(this->invoke_requests);
  lock.unlock();
  for (auto& request : invoke_requests) {
    request();
  }

  // To allow removing a callback from a callback call, avoid directly iterating over ui_callbacks
  std::vector<const std::function<void()>*> callbacks;
  for (const auto& callback : ui_callbacks) {
    callbacks.emplace_back(&callback.second);
  }
  for (const auto& callback : callbacks) {
    (*callback)();
  }

  // viewer UI
  if (viewer_ui) {
    if (!viewer_ui->draw_ui()) {
      viewer_ui.reset();
    }
  } else {
    if (ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeysDown[GLFW_KEY_M]) {
      viewer_ui.reset(new ViewerUI(this));
    }
  }

  // Fit plots
  if (ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeysDown[GLFW_KEY_F]) {
    fit_all_plots();
  }

  // point scale
  bool decrease_point_size = ImGui::GetIO().KeysDown[GLFW_KEY_MINUS];
  bool increase_point_size = ImGui::GetIO().KeyShift && ImGui::GetIO().KeysDown[GLFW_KEY_SEMICOLON];
  if (decrease_point_size || increase_point_size) {
    auto point_size = global_shader_setting.get<float>("point_size");
    if (!point_size) {
      point_size = 0.025f;
    }

    const auto point_scale_mode = global_shader_setting.get<int>("point_scale_mode");
    const float scaling_factor = (point_scale_mode && *point_scale_mode == guik::PointScaleMode::SCREENSPACE) ? 10.0f : 0.1f;

    if (decrease_point_size) {
      *point_size = point_size.value() - ImGui::GetIO().DeltaTime * scaling_factor;
    } else {
      *point_size = point_size.value() + ImGui::GetIO().DeltaTime * scaling_factor;
    }

    *point_size = std::max(1e-4f, std::min(1e6f, point_size.value()));

    global_shader_setting.add("point_size", *point_size);
  }

  // screen shot
  if (ImGui::GetIO().KeysDown[GLFW_KEY_J]) {
    invoke_after_rendering([this] {
      auto bytes = canvas->frame_buffer->color().read_pixels<unsigned char>(GL_RGBA, GL_UNSIGNED_BYTE, 4);
      std::vector<unsigned char> flipped(bytes.size(), 255);

      Eigen::Vector2i size = canvas->frame_buffer->color().size();
      for (int y = 0; y < size[1]; y++) {
        int y_ = size[1] - y - 1;
        for (int x = 0; x < size[0]; x++) {
          for (int k = 0; k < 3; k++) {
            flipped[(y_ * size[0] + x) * 4 + k] = bytes[(y * size[0] + x) * 4 + k];
          }
        }
      }

      double time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() / 1e9;
      std::string filename = "/tmp/ss_" + std::to_string(static_cast<int>(time)) + ".png";
      if (glk::save_png(filename, canvas->size[0], canvas->size[1], flipped)) {
        std::cout << "screen shot saved:" << filename << std::endl;
      } else {
        std::cout << "failed to save screen shot" << std::endl;
      }
    });
  }

  if (info_window) {
    if (!info_window->draw_ui()) {
      info_window.reset();
    }
  }

  // texts
  std::vector<std::string> texts_;
  {
    std::lock_guard<std::mutex> texts_lock(texts_mutex);
    std::vector<std::string>(texts.begin(), texts.end()).swap(texts_);
  }

  if (!texts_.empty()) {
    ImGui::Begin(
      "texts",
      nullptr,
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    for (int i = std::max<int>(0, texts_.size() - max_texts_size); i < texts_.size(); i++) {
      const auto& text = texts_[i];
      ImGui::Text("%s", text.c_str());
    }

    if (ImGui::Button("clear")) {
      clear_text();
    }
    ImGui::End();
  }

  // images
  if (!images.empty()) {
    ImGui::Begin("images", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    bool grouping = false;
    images_in_rendering.clear();
    std::unordered_map<std::string, std::vector<std::string>> groups;
    for (const auto& image : images) {
      const size_t separator_loc = image.first.find_first_of('/');
      grouping |= (separator_loc != std::string::npos);

      const std::string group = separator_loc == std::string::npos ? "default" : image.first.substr(0, separator_loc);
      groups[group].push_back(image.first);
      images_in_rendering.emplace_back(std::get<1>(image.second));
    }

    if (grouping) {
      ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
      ImGui::BeginTabBar("imagetab", tab_bar_flags);
    }

    for (auto& group : groups) {
      if (grouping) {
        if (!ImGui::BeginTabItem(group.first.c_str())) {
          continue;
        }
      }

      const auto& group_name = group.first;
      auto& image_names = group.second;
      std::sort(image_names.begin(), image_names.end(), [this](const auto& lhs, const auto& rhs) { return std::get<2>(images[lhs]) < std::get<2>(images[rhs]); });

      for (const auto& name : image_names) {
        const auto& image = images[name];
        const double scale = std::get<0>(image);
        const auto& texture = std::get<1>(image);

        Eigen::Vector2i size = (texture->size().cast<double>() * scale).cast<int>();

        ImGui::Text("%s", name.c_str());
        ImGui::Image(reinterpret_cast<void*>(texture->id()), ImVec2(size[0], size[1]), ImVec2(0, 0), ImVec2(1, 1));
      }

      if (grouping) {
        ImGui::EndTabItem();
      }
    }

    if (grouping) {
      ImGui::EndTabBar();
    }

    ImGui::End();
  }

  // plots
  if (!plot_data.empty()) {
    ImGui::Begin("plots", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    bool grouping = false;
    std::vector<std::pair<std::string, std::vector<std::string>>> groups;
    for (const auto& plot : plot_data) {
      const size_t separator_loc = plot.first.find_first_of('/');
      grouping |= (separator_loc != std::string::npos);

      const std::string group = separator_loc == std::string::npos ? "default" : plot.first.substr(0, separator_loc);
      auto found = std::find_if(groups.begin(), groups.end(), [&group](const auto& g) { return g.first == group; });
      if (found == groups.end()) {
        groups.emplace_back(group, std::vector<std::string>());
        found = groups.end() - 1;
      }

      found->second.push_back(plot.first);
    }

    if (grouping) {
      ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
      ImGui::BeginTabBar("plottab", tab_bar_flags);

      if (!plot_group_orders.empty()) {
        std::sort(groups.begin(), groups.end(), [this](const auto& lhs, const auto& rhs) { return plot_group_orders[lhs.first] < plot_group_orders[rhs.first]; });
      }
    }

    for (auto& group : groups) {
      if (grouping) {
        if (!ImGui::BeginTabItem(group.first.c_str())) {
          continue;
        }
      }

      std::sort(group.second.begin(), group.second.end(), [this](const auto& lhs, const auto& rhs) { return plot_settings[lhs].order < plot_settings[rhs].order; });

      for (const auto& plot_name : group.second) {
        auto& plot_setting = plot_settings[plot_name];
        if (plot_setting.set_axes_to_fit) {
          ImPlot::SetNextAxesToFit();
          plot_setting.set_axes_to_fit = false;
        }

        const auto& plots = plot_data[plot_name];
        if (ImPlot::BeginPlot(plot_name.c_str(), ImVec2(plot_setting.width, plot_setting.height), plot_setting.plot_flags)) {
          if (!plots.empty()) {
            const auto& plot = plots.front();

            if (plot_setting.axis_link_id >= 0) {
              auto& limits = plot_linked_axis_limits[plot_setting.axis_link_id];

              for (int axis = 0; axis < ImAxis_COUNT; axis++) {
                if (plot_setting.linked_axes & (1 << axis)) {
                  ImPlot::SetupAxisLinks(axis, &limits(axis, 0), &limits(axis, 1));
                }
              }
            }

            ImPlot::SetupAxes(plot_setting.x_label.c_str(), plot_setting.y_label.c_str(), plot_setting.x_flags, plot_setting.y_flags);
            ImPlot::SetupLegend(plot_setting.legend_loc, plot_setting.legend_flags);
          }

          for (const auto& plot : plots) {
            if (!plot.second) {
              continue;
            }

            if (plot.first) {
              plot.first->apply();
            }
            plot.second->plot();
          }

          ImPlot::EndPlot();
        }
      }

      if (grouping) {
        ImGui::EndTabItem();
      }
    }

    if (grouping) {
      ImGui::EndTabBar();
    }

    ImGui::End();
  }

  // mouse control
  if (!ImGui::GetIO().WantCaptureMouse) {
    canvas->mouse_control();
  }

  for (const auto& context : sub_contexts) {
    context.second->draw_gl();
    context.second->draw_ui();
  }
}

void LightViewer::draw_gl() {
  LightViewerContext::draw_gl();
  canvas->render_to_screen();

  std::unique_lock<std::mutex> lock(post_render_invoke_requests_mutex);
  std::deque<std::function<void()>> post_render_invoke_requests;
  post_render_invoke_requests.swap(this->post_render_invoke_requests);
  lock.unlock();

  for (auto& request : post_render_invoke_requests) {
    request();
  }
}

void LightViewer::clear() {
  clear_text();
  invoke_requests_mutex.lock();
  invoke_requests.clear();
  invoke_requests_mutex.unlock();
  post_render_invoke_requests_mutex.lock();
  post_render_invoke_requests.clear();
  post_render_invoke_requests_mutex.unlock();
  ui_callbacks.clear();
  sub_contexts.clear();
  clear_drawables();
}

void LightViewer::clear_text() {
  std::lock_guard<std::mutex> lock(texts_mutex);
  texts.clear();
}

void LightViewer::append_text(const std::string& text) {
  std::vector<std::string> texts = glk::split_lines(text);

  std::lock_guard<std::mutex> lock(texts_mutex);
  this->texts.insert(this->texts.end(), texts.begin(), texts.end());
}

void LightViewer::set_max_text_buffer_size(int size) {
  max_texts_size = size;
}

void LightViewer::clear_images() {
  images.clear();
}

void LightViewer::remove_image(const std::string& name) {
  auto found = images.find(name);
  if (found != images.end()) {
    images.erase(found);
  }
}

void LightViewer::update_image(const std::string& name, const std::shared_ptr<glk::Texture>& image, double scale, int order) {
  if (!image) {
    remove_image(name);
    return;
  }

  if (scale < 0.0) {
    double scale_x = 640.0 / image->size()[0];
    double scale_y = 480.0 / image->size()[1];
    scale = std::min(scale_x, scale_y);
  }

  images[name] = std::make_tuple(scale, image, order >= 0 ? order : 8192 + images.size());
}

void LightViewer::clear_plots(bool clear_settings) {
  if (clear_settings) {
    plot_settings.clear();
  }
  plot_data.clear();
}

void LightViewer::remove_plot(const std::string& plot_name, const std::string& label) {
  auto found = plot_data.find(plot_name);
  if (found == plot_data.end()) {
    return;
  }

  if (label.empty()) {
    plot_data.erase(found);
    return;
  }

  auto& data = found->second;
  data.erase(std::find_if(data.begin(), data.end(), [&](const auto& p) { return p.second->label == label; }));
}

void LightViewer::setup_plot(const std::string& plot_name, int width, int height, int plot_flags, int x_flags, int y_flags, int order) {
  auto& setting = plot_settings[plot_name];
  setting.width = width;
  setting.height = height;
  setting.plot_flags = plot_flags;
  setting.x_flags = x_flags;
  setting.y_flags = y_flags;
  setting.order = order >= 0 ? order : 8192 + plot_settings.size();
}

void LightViewer::link_plot_axis(const std::string& plot_name, int link_id, int axis) {
  auto& setting = plot_settings[plot_name];
  setting.axis_link_id = link_id;
  setting.linked_axes |= (1 << axis);

  if (plot_linked_axis_limits.count(link_id) == 0) {
    plot_linked_axis_limits[link_id] = Eigen::Matrix<double, 6, 2>::Zero();
  }
}

void LightViewer::link_plot_axes(const std::string& plot_name, int link_id, int axes) {
  auto& setting = plot_settings[plot_name];
  setting.axis_link_id = link_id;
  setting.linked_axes = axes;

  if (plot_linked_axis_limits.count(link_id) == 0) {
    plot_linked_axis_limits[link_id] = Eigen::Matrix<double, 6, 2>::Zero();
  }
}

void LightViewer::setup_legend(const std::string& plot_name, int loc, int flags) {
  auto& setting = plot_settings[plot_name];
  setting.legend_loc = loc;
  setting.legend_flags = flags;
}

void LightViewer::fit_plot(const std::string& plot_name) {
  plot_settings[plot_name].set_axes_to_fit = true;
}

void LightViewer::fit_all_plots() {
  for (auto setting = plot_settings.begin(); setting != plot_settings.end(); setting++) {
    setting->second.set_axes_to_fit = true;
  }
}

void LightViewer::setup_plot_group_order(const std::string& group_name, int order) {
  plot_group_orders[group_name] = order;
}

void LightViewer::update_plot(const std::string& plot_name, const std::string& label, const std::shared_ptr<const PlotData>& plot) {
  auto& data = plot_data[plot_name];
  auto found = std::find_if(data.begin(), data.end(), [&](const auto& p_) { return p_.second->label == plot->label; });
  if (found == data.end()) {
    data.emplace_back(nullptr, plot);
  } else {
    (*found).second = plot;
  }
}

void LightViewer::update_plot_line(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int line_flags, size_t max_num_data) {
  std::vector<double> xs(ys.size());
  std::iota(xs.begin(), xs.end(), 0.0);
  update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
}

void LightViewer::update_plot_line(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  int line_flags,
  size_t max_num_data) {
  if (xs.size() != ys.size()) {
    std::cerr << "warning: the length of xs must be the same as the length of ys (" << xs.size() << " vs " << ys.size() << ")" << std::endl;
    return;
  }

  auto p = std::make_shared<LinePlotData>(label);
  p->line_flags = line_flags;

  if (xs.size() <= max_num_data) {
    p->xs = xs;
    p->ys = ys;
  } else {
    p->xs.resize(max_num_data);
    p->ys.resize(max_num_data);

    for (int i = 0; i < max_num_data; i++) {
      const double t = static_cast<double>(i) / max_num_data;
      const size_t j = std::min<size_t>(t * xs.size(), xs.size() - 1);

      p->xs[i] = xs[j];
      p->ys[i] = ys[j];
    }
  }

  update_plot(plot_name, label, p);
}

void LightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int scatter_flags) {
  std::vector<double> xs(ys.size());
  std::iota(xs.begin(), xs.end(), 0.0);
  update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
}

void LightViewer::update_plot_scatter(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags) {
  auto p = std::make_shared<ScatterPlotData>(label);
  p->scatter_flags = scatter_flags;
  p->xs = xs;
  p->ys = ys;

  update_plot(plot_name, label, p);
}

void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& ys, int stairs_flags) {
  std::vector<double> xs(ys.size());
  std::iota(xs.begin(), xs.end(), 0.0);
  update_plot_stairs(plot_name, label, xs, ys, stairs_flags);
}

void LightViewer::update_plot_stairs(const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags) {
  auto p = std::make_shared<StairsPlotData>(label);
  p->stairs_flags = stairs_flags;
  p->xs = xs;
  p->ys = ys;

  update_plot(plot_name, label, p);
}

void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  int bins,
  const Eigen::Vector2d& range,
  int histogram_flags) {
  //
  auto p = std::make_shared<HistogramPlotData>(label);
  p->histogram_flags = histogram_flags;
  p->x_bins = bins;
  p->x_range_min = range[0];
  p->x_range_max = range[1];
  p->xs = xs;

  update_plot(plot_name, label, p);
}

void LightViewer::update_plot_histogram(
  const std::string& plot_name,
  const std::string& label,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  int x_bins,
  int y_bins,
  const Eigen::Vector2d& x_range,
  const Eigen::Vector2d& y_range,
  int histogram_flags) {
  //
  auto p = std::make_shared<HistogramPlotData>(label);
  p->histogram_flags = histogram_flags;
  p->x_bins = x_bins;
  p->y_bins = y_bins;
  p->x_range_min = x_range[0];
  p->x_range_max = x_range[1];
  p->xs = xs;
  p->ys = ys;

  update_plot(plot_name, label, p);
}

void LightViewer::set_plot_style(const std::string& plot_name, const std::string& label, const PlotStyleConstPtr& style) {
  auto& data = plot_data[plot_name];
  auto found = std::find_if(data.begin(), data.end(), [&](const auto& p_) { return p_.second->label == label; });
  if (found == data.end()) {
    data.emplace_back(style, std::make_shared<PlotData>(label));
  } else {
    (*found).first = style;
  }
}

void LightViewer::set_line_style(const std::string& plot_name, const std::string& label, const Eigen::Vector4f& color, float weight) {
  auto s = std::make_shared<LinePlotStyle>();
  s->col = color;
  s->weight = weight;

  set_plot_style(plot_name, label, s);
}

void LightViewer::set_scatter_style(
  const std::string& plot_name,
  const std::string& label,
  int marker,
  float size,
  const Eigen::Vector4f& fill,
  float weight,
  const Eigen::Vector4f& outline) {
  auto s = std::make_shared<ScatterPlotStyle>();
  s->marker = marker;
  s->size = size;
  s->fill = fill;
  s->weight = weight;
  s->outline = outline;

  set_plot_style(plot_name, label, s);
}

bool LightViewer::spin_until_click() {
  bool kill_switch = false;

  register_ui_callback("kill_switch", [&]() { kill_switch = ImGui::Button("break"); });

  while (!kill_switch) {
    if (!spin_once()) {
      return false;
    }
  }

  register_ui_callback("kill_switch", nullptr);

  return true;
}

bool LightViewer::toggle_spin_once() {
  bool step = false;
  register_ui_callback("kill_switch", [&]() {
    ImGui::Checkbox("break", &toggle_spin_stop_flag);
    ImGui::SameLine();
    if (ImGui::Button("step")) {
      step = true;
    }
  });

  do {
    if (!spin_once()) {
      return false;
    }
  } while (toggle_spin_stop_flag && !step);

  register_ui_callback("kill_switch", nullptr);

  return true;
}

void LightViewer::register_ui_callback(const std::string& name, const std::function<void()>& callback) {
  if (!callback) {
    ui_callbacks.erase(name);
    return;
  }

  ui_callbacks[name] = callback;
}

void LightViewer::show_viewer_ui() {
  if (viewer_ui == nullptr) {
    viewer_ui.reset(new ViewerUI(this));
  }
}

void LightViewer::show_info_window() {
  if (info_window == nullptr) {
    info_window.reset(new InfoWindow());
  }
}

void LightViewer::invoke(const std::function<void()>& func) {
  std::lock_guard<std::mutex> lock(invoke_requests_mutex);
  invoke_requests.push_back(func);
}

void LightViewer::invoke_after_rendering(const std::function<void()>& func) {
  std::lock_guard<std::mutex> lock(post_render_invoke_requests_mutex);
  post_render_invoke_requests.push_back(func);
}

void LightViewer::invoke_once(const std::string& label, const std::function<void()>& func) {
  std::lock_guard<std::mutex> lock(invoke_requests_mutex);
  if (invoke_once_called.count(label) == 0) {
    invoke_requests.push_back(func);
    invoke_once_called.insert(label);
  }
}

std::shared_ptr<LightViewerContext> LightViewer::sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size) {
  using namespace glk::console;

  auto found = sub_contexts.find(context_name);
  if (found == sub_contexts.end()) {
    Eigen::Vector2i init_canvas_size = canvas_size;
    if (canvas_size[0] <= 0 || canvas_size[1] <= 0) {
      init_canvas_size = Eigen::Vector2i(512, 512);
    }

    std::shared_ptr<LightViewerContext> context(new LightViewerContext(context_name));
    if (!context->init_canvas(init_canvas_size)) {
      std::cerr << bold_red << "error: failed to create sub viewer context!!" << reset << std::endl;
      return nullptr;
    }

    sub_contexts[context_name] = context;
    return context;
  }

  if ((canvas_size.array() > 0).all() && found->second->canvas_size() != canvas_size) {
    if (!found->second->init_canvas(canvas_size)) {
      std::cerr << bold_red << "error: failed to resize the canvas of " << context_name << "!!" << reset << std::endl;
      close();
    }
  }

  return found->second;
}

void LightViewer::show_sub_viewers() {
  for (auto& sub : sub_contexts) {
    sub.second->show();
  }
}

std::shared_ptr<LightViewerContext> LightViewer::find_sub_viewer(const std::string& context_name) {
  auto found = sub_contexts.find(context_name);
  return found != sub_contexts.end() ? found->second : nullptr;
}

bool LightViewer::remove_sub_viewer(const std::string& context_name) {
  using namespace glk::console;

  auto found = sub_contexts.find(context_name);
  if (found == sub_contexts.end()) {
    std::cerr << yellow << "warning: sub viewer (context_name=" << context_name << ") not found!" << reset << std::endl;
    return false;
  }
  sub_contexts.erase(found);
  return true;
}

}  // namespace guik
