#include <guik/viewer/light_viewer_context.hpp>

#include <fstream>
#include <ImGuizmo.h>
#include <glk/io/png_io.hpp>
#include <glk/console_colors.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer_context.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/camera/arcball_camera_control.hpp>
#include <guik/camera/fps_camera_control.hpp>
#include <guik/camera/basic_projection_control.hpp>

namespace guik {

LightViewerContext::LightViewerContext(const std::string& context_name) : context_name(context_name) {
  show_window = true;
  draw_xy_grid = true;
  decimal_rendering = false;
  last_projection_view_matrix.setIdentity();
}

LightViewerContext::~LightViewerContext() {}

bool LightViewerContext::init_canvas(const Eigen::Vector2i& size) {
  canvas_rect_min.setZero();
  canvas_rect_max = size;
  canvas.reset(new guik::GLCanvas(size));
  if (!canvas->ready()) {
    return false;
  }

  global_shader_setting.add("model_matrix", Eigen::Matrix4f::Identity().eval());

  return true;
}

guik::GLCanvas& LightViewerContext::get_canvas() {
  return *canvas;
}

void LightViewerContext::set_size(const Eigen::Vector2i& size) {
  canvas->set_size(size);
}

void LightViewerContext::set_clear_color(const Eigen::Vector4f& color) {
  canvas->set_clear_color(color);
}

void LightViewerContext::set_pos(const Eigen::Vector2i& pos, ImGuiCond cond, ImGuiWindowFlags flags) {
  using namespace glk::console;
  if (context_name == "main") {
    std::cout << yellow << "warning: calling set_pos() is valid for only sub viewers" << reset << std::endl;
  }

  int x = pos[0];
  int y = pos[1];

  guik::LightViewer::instance()->invoke([=] {
    ImGui::SetNextWindowPos(ImVec2(x, y), cond);
    ImGui::Begin(context_name.c_str(), nullptr, flags);
    ImGui::End();
  });
}

void LightViewerContext::show() {
  using namespace glk::console;
  if (context_name == "main") {
    std::cout << yellow << "warning: calling show() is valid for only sub viewers" << reset << std::endl;
  }

  show_window = true;
}

void LightViewerContext::hide() {
  using namespace glk::console;
  if (context_name == "main") {
    std::cout << yellow << "warning: calling hide() is valid for only sub viewers" << reset << std::endl;
  }

  show_window = false;
}

void LightViewerContext::clear() {
  sub_ui_callbacks.clear();
  drawable_filters.clear();
  drawables.clear();

  std::lock_guard<std::mutex> lock(sub_texts_mutex);
  sub_texts.clear();
}

void LightViewerContext::clear_text() {
  std::lock_guard<std::mutex> lock(sub_texts_mutex);
  sub_texts.clear();
}

void LightViewerContext::append_text(const std::string& text) {
  const auto split_lines = [](const std::string& text) {
    std::vector<std::string> tokens;

    size_t loc = 0;
    size_t found = 0;

    do {
      found = text.find_first_of('\n', loc);
      tokens.push_back(text.substr(loc, found - loc));
      loc = found + 1;
    } while (found != std::string::npos);

    return tokens;
  };

  std::vector<std::string> texts = split_lines(text);

  std::lock_guard<std::mutex> lock(sub_texts_mutex);
  sub_texts.insert(sub_texts.end(), texts.begin(), texts.end());
}

void LightViewerContext::register_ui_callback(const std::string& name, const std::function<void()>& callback) {
  if (!callback) {
    sub_ui_callbacks.erase(name);
    return;
  }

  sub_ui_callbacks[name] = callback;
}

void LightViewerContext::remove_ui_callback(const std::string& name) {
  register_ui_callback(name, 0);
}

void LightViewerContext::draw_ui() {
  if (!show_window) {
    return;
  }

  ImGui::Begin(context_name.c_str(), &show_window, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBringToFrontOnFocus);

  ImGuiWindowFlags flags = ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavFocus;
  ImGui::BeginChild("canvas", ImVec2(canvas->size[0], canvas->size[1]), false, flags);
  ImVec2 sub_window_pos = ImGui::GetWindowPos();

  if (ImGui::IsWindowFocused() && !ImGuizmo::IsUsing()) {
    canvas->mouse_control();
  }

  ImGui::Image(reinterpret_cast<void*>(canvas->frame_buffer->color().id()), ImVec2(canvas->size[0], canvas->size[1]), ImVec2(0, 1), ImVec2(1, 0));
  ImVec2 rect_min = ImGui::GetItemRectMin();
  ImVec2 rect_max = ImGui::GetItemRectMax();
  canvas_rect_min = Eigen::Vector2i(rect_min.x, rect_min.y);
  canvas_rect_max = Eigen::Vector2i(rect_max.x, rect_max.y);

  std::vector<const std::function<void()>*> callbacks;
  for (const auto& callback : sub_ui_callbacks) {
    callbacks.emplace_back(&callback.second);
  }

  for (const auto& callback : callbacks) {
    (*callback)();
  }

  ImGui::EndChild();

  ImGui::End();

  std::vector<std::string> texts_;
  {
    std::lock_guard<std::mutex> texts_lock(sub_texts_mutex);
    std::vector<std::string>(sub_texts.begin(), sub_texts.end()).swap(texts_);
  }

  if (!texts_.empty()) {
    std::string window_name = "sub_texts_" + context_name;
    ImGui::SetNextWindowPos(ImVec2(sub_window_pos.x + 5, sub_window_pos.y + 5), ImGuiCond_Always);
    ImGui::Begin(
      window_name.c_str(),
      nullptr,
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    for (int i = std::max<int>(0, texts_.size() - 28); i < texts_.size(); i++) {
      const auto& text = texts_[i];
      ImGui::Text("%s", text.c_str());
    }

    if (ImGui::Button("clear")) {
      clear_text();
    }
    ImGui::End();
  }
}

void LightViewerContext::draw_gl() {
  std::vector<std::pair<guik::ShaderSetting::Ptr, glk::Drawable::ConstPtr>> active_drawables;
  for (const auto& itr : drawables) {
    bool draw = true;
    for (const auto& filter : drawable_filters) {
      if (!filter.second(itr.first)) {
        draw = false;
        break;
      }
    }

    const auto& drawable = itr.second.second;
    if (draw && drawable) {
      active_drawables.push_back(itr.second);
    }
  }

  canvas->bind();

  global_shader_setting.set(*canvas->shader);
  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
  canvas->shader->set_uniform("color_mode", 1);
  if (draw_xy_grid) {
    canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.6f, 0.6f, 0.6f, 1.0f));
    glk::Primitives::instance()->primitive(glk::Primitives::GRID).draw(*canvas->shader);
  }

  bool transparent_exists = false;
  for (const auto& drawable : active_drawables) {
    if (!drawable.first->transparent) {
      drawable.first->set(*canvas->shader);
      drawable.second->draw(*canvas->shader);
    } else {
      transparent_exists = true;
    }
  }

  canvas->unbind();

  if (transparent_exists) {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    canvas->bind_second();

    global_shader_setting.set(*canvas->shader);
    canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
    canvas->shader->set_uniform("color_mode", 1);

    for (const auto& drawable : active_drawables) {
      if (drawable.first->transparent) {
        drawable.first->set(*canvas->shader);
        drawable.second->draw(*canvas->shader);
      }
    }

    canvas->unbind_second();

    glDisable(GL_CULL_FACE);
  }
}

void LightViewerContext::lookat(const Eigen::Vector3f& pt) {
  canvas->camera_control->lookat(pt);
}

void LightViewerContext::set_draw_xy_grid(bool draw_xy_grid) {
  this->draw_xy_grid = draw_xy_grid;
}

void LightViewerContext::set_colormap(glk::COLORMAP colormap) {
  canvas->set_colormap(colormap);
}

void LightViewerContext::set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
  canvas->set_effect(effect);
}

const std::shared_ptr<glk::ScreenEffect>& LightViewerContext::get_screen_effect() const {
  return canvas->get_effect();
}

void LightViewerContext::set_bg_texture(const std::shared_ptr<glk::Texture>& bg_texture) {
  canvas->set_bg_texture(bg_texture);
}

void LightViewerContext::set_rainbow_range(const Eigen::Vector2f& minmax_z) {
  shader_setting().set_rainbow_range(minmax_z);
}

void LightViewerContext::set_rainbow_axis(const Eigen::Vector3f& axis) {
  shader_setting().set_rainbow_axis(axis);
}

void LightViewerContext::set_point_shape(float point_size, bool metric, bool circle) {
  shader_setting().set_point_shape(point_size, metric, circle);
}

void LightViewerContext::enable_decimal_rendering() {
  decimal_rendering = true;
}

void LightViewerContext::enable_normal_buffer() {
  canvas->enable_normal_buffer();
}

void LightViewerContext::enable_info_buffer() {
  canvas->enable_info_buffer();
}

void LightViewerContext::enable_partial_rendering(double clear_thresh) {
  canvas->enable_partial_rendering(clear_thresh);
}

void LightViewerContext::disable_partial_rendering() {
  canvas->disable_partial_rendering();
}

bool LightViewerContext::normal_buffer_enabled() const {
  return canvas->normal_buffer_enabled();
}

bool LightViewerContext::info_buffer_enabled() const {
  return canvas->info_buffer_enabled();
}

bool LightViewerContext::partial_rendering_enabled() const {
  return canvas->partial_rendering_enabled();
}

const glk::Texture& LightViewerContext::color_buffer() const {
  return canvas->color_buffer();
}

const glk::Texture& LightViewerContext::depth_buffer() const {
  return canvas->depth_buffer();
}

const glk::Texture& LightViewerContext::normal_buffer() const {
  return canvas->normal_buffer();
}

const glk::Texture& LightViewerContext::info_buffer() const {
  return canvas->info_buffer();
}

const glk::Texture& LightViewerContext::dynamic_flag_buffer() const {
  return canvas->dynamic_flag_buffer();
}

void LightViewerContext::clear_drawables() {
  drawables.clear();
}

void LightViewerContext::clear_drawables(const std::function<bool(const std::string&)>& fn) {
  for (auto it = drawables.begin(); it != drawables.end();) {
    if (fn(it->first)) {
      it = drawables.erase(it);
    } else {
      it++;
    }
  }
}

std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>>& LightViewerContext::get_drawables() {
  return drawables;
}

std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr> LightViewerContext::find_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if (found != drawables.end()) {
    return found->second;
  }
  return std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>();
}

void LightViewerContext::remove_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if (found != drawables.end()) {
    drawables.erase(found);
  }
}

void LightViewerContext::remove_drawable(const std::regex& regex) {
  for (auto itr = drawables.begin(); itr != drawables.end();) {
    if (std::regex_match(itr->first, regex)) {
      itr = drawables.erase(itr);
    } else {
      itr++;
    }
  }
}

void LightViewerContext::update_drawable(const std::string& name, const glk::Drawable::ConstPtr& drawable, const ShaderSetting& shader_setting) {
  drawables[name] = std::make_pair(std::make_shared<ShaderSetting>(shader_setting), drawable);
}

void LightViewerContext::clear_drawable_filters() {
  drawable_filters.clear();
}

void LightViewerContext::register_drawable_filter(const std::string& filter_name, const std::function<bool(const std::string&)>& filter) {
  if (!filter) {
    auto found = drawable_filters.find(filter_name);
    if (found != drawable_filters.end()) {
      drawable_filters.erase(found);
    }
    return;
  }

  drawable_filters[filter_name] = filter;
}

void LightViewerContext::remove_drawable_filter(const std::string& filter_name) {
  register_drawable_filter(filter_name, 0);
}

void LightViewerContext::clear_partial_rendering() {
  canvas->clear_partial_rendering();
}

const std::shared_ptr<CameraControl>& LightViewerContext::get_camera_control() const {
  return canvas->camera_control;
}

const std::shared_ptr<ProjectionControl>& LightViewerContext::get_projection_control() const {
  return canvas->projection_control;
}

void LightViewerContext::set_camera_control(const std::shared_ptr<CameraControl>& camera_control) {
  canvas->camera_control = camera_control;
}

void LightViewerContext::set_projection_control(const std::shared_ptr<ProjectionControl>& projection_control) {
  canvas->projection_control = projection_control;
}

bool LightViewerContext::save_camera_settings(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs) {
    std::cerr << glk::console::red << "error: failed to open " << path << " for writing" << glk::console::reset << std::endl;
    return false;
  }

  auto projection = this->get_projection_control();
  ofs << "ProjectionControl: " << projection->name() << std::endl;
  ofs << (*projection) << std::endl;

  auto view = this->get_camera_control();
  ofs << "CameraControl: " << view->name() << std::endl;
  ofs << (*view) << std::endl;
  return true;
}

bool LightViewerContext::load_camera_settings(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << glk::console::red << "error: failed to open " << path << " for reading" << glk::console::reset << std::endl;
    return false;
  }

  // load projection setting
  std::shared_ptr<guik::ProjectionControl> proj(new guik::BasicProjectionControl(this->canvas_size()));
  ifs >> (*proj);
  this->set_projection_control(proj);

  std::string line;
  while (!ifs.eof() && std::getline(ifs, line)) {
    if (line.find("CameraControl") == std::string::npos) {
      continue;
    }

    std::stringstream sst(line);
    std::string token, type;
    sst >> token >> type;

    std::shared_ptr<guik::CameraControl> camera_control;
    if (type == "OrbitCameraControlXY") {
      camera_control.reset(new guik::OrbitCameraControlXY());
    } else if (type == "OrbitCameraControlXZ") {
      camera_control.reset(new guik::OrbitCameraControlXZ());
    } else if (type == "TopDownCameraControl") {
      camera_control.reset(new guik::TopDownCameraControl());
    } else if (type == "ArcBallCameraControl") {
      camera_control.reset(new guik::ArcBallCameraControl());
    }

    if (camera_control == nullptr) {
      std::cerr << "error: unknown camera control type(" << type << ")" << std::endl;
      break;
    }

    ifs >> (*camera_control);
    this->set_camera_control(camera_control);
  }

  return true;
}

void LightViewerContext::reset_center() {
  canvas->camera_control->reset_center();
}

std::shared_ptr<OrbitCameraControlXY> LightViewerContext::use_orbit_camera_control(double distance, double theta, double phi) {
  auto camera = std::make_shared<guik::OrbitCameraControlXY>(distance, theta, phi);
  canvas->camera_control = camera;
  return camera;
}

std::shared_ptr<OrbitCameraControlXZ> LightViewerContext::use_orbit_camera_control_xz(double distance, double theta, double phi) {
  auto camera = std::make_shared<guik::OrbitCameraControlXZ>(distance, theta, phi);
  canvas->camera_control = camera;
  return camera;
}

std::shared_ptr<TopDownCameraControl> LightViewerContext::use_topdown_camera_control(double distance, double theta) {
  auto camera = std::make_shared<guik::TopDownCameraControl>(distance, theta);
  canvas->camera_control = camera;
  return camera;
}

std::shared_ptr<ArcBallCameraControl> LightViewerContext::use_arcball_camera_control(double distance, double theta, double phi) {
  Eigen::Quaternionf quat(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));
  auto camera = std::make_shared<guik::ArcBallCameraControl>(distance, quat);
  canvas->camera_control = camera;
  return camera;
}

std::shared_ptr<FPSCameraControl> LightViewerContext::use_fps_camera_control(double fovy_deg) {
  auto fps_camera_control = std::make_shared<guik::FPSCameraControl>(canvas->size);
  fps_camera_control->set_fovy(fovy_deg);
  canvas->camera_control = fps_camera_control;
  canvas->projection_control = fps_camera_control;
  return fps_camera_control;
}

Eigen::Vector4i LightViewerContext::pick_info(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_info(p, window);
}

float LightViewerContext::pick_depth(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_depth(p, window);
}

Eigen::Vector3f LightViewerContext::unproject(const Eigen::Vector2i& p, float depth) const {
  return canvas->unproject(p, depth);
}

std::optional<Eigen::Vector3f> LightViewerContext::pick_point(int button, int window, Eigen::Vector4i* info) const {
  const auto& io = ImGui::GetIO();
  if (io.WantCaptureMouse || !io.MouseClicked[button]) {
    return std::nullopt;
  }

  float depth = pick_depth({io.MousePos.x, io.MousePos.y}, window);
  if (depth >= 1.0f) {
    return std::nullopt;
  }

  if (info) {
    *info = pick_info({io.MousePos.x, io.MousePos.y}, window);
  }

  return unproject({io.MousePos.x, io.MousePos.y}, depth);
}

std::vector<unsigned char> LightViewerContext::read_color_buffer() const {
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

  return flipped;
}

std::vector<float> LightViewerContext::read_depth_buffer(bool real_scale) {
  auto floats = canvas->frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT, 1);
  std::vector<float> flipped(floats.size());

  Eigen::Vector2i size = canvas->frame_buffer->depth().size();
  for (int y = 0; y < size[1]; y++) {
    int y_ = size[1] - y - 1;
    for (int x = 0; x < size[0]; x++) {
      flipped[y_ * size[0] + x] = floats[y * size[0] + x];
    }
  }

  if (real_scale) {
    const Eigen::Vector2f depth_range = canvas->camera_control->depth_range();
    const float near_ = depth_range[0];
    const float far_ = depth_range[1];
    for (auto& depth : flipped) {
      depth = 2.0 * near_ * far_ / (far_ + near_ - depth * (far_ - near_));
    }
  }

  return flipped;
}

bool LightViewerContext::save_color_buffer(const std::string& filename) {
  auto bytes = this->read_color_buffer();
  if (glk::save_png(filename, canvas->size[0], canvas->size[1], bytes)) {
  } else {
    std::cout << "warning : failed to save screen shot" << std::endl;
    return false;
  }

  return true;
}

bool LightViewerContext::save_depth_buffer(const std::string& filename, bool real_scale) {
  auto depths = this->read_depth_buffer(real_scale);

  std::vector<unsigned char> depths_u8(sizeof(float) * depths.size());
  memcpy(depths_u8.data(), depths.data(), sizeof(float) * depths.size());

  if (glk::save_png(filename, canvas->size[0], canvas->size[1], depths_u8)) {
  } else {
    std::cout << "warning : failed to save depth buffer" << std::endl;
    return false;
  }

  return true;
}

AsyncLightViewerContext LightViewerContext::async() {
  return AsyncLightViewerContext(this);
}

}  // namespace guik
