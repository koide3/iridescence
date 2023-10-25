#include <guik/viewer/light_viewer_context.hpp>

#include <boost/algorithm/string.hpp>

#include <ImGuizmo.h>
#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/camera/arcball_camera_control.hpp>

namespace guik {

LightViewerContext::LightViewerContext(const std::string& context_name) : context_name(context_name) {
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

void LightViewerContext::set_pos(const Eigen::Vector2i& pos, ImGuiCond cond) {
  int x = pos[0];
  int y = pos[1];

  guik::LightViewer::instance()->invoke([=] {
    ImGui::SetNextWindowPos(ImVec2(x, y), cond);
    ImGui::Begin(context_name.c_str());
    ImGui::End();
  });
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
  std::vector<std::string> texts;
  boost::split(texts, text, boost::is_any_of("\n"));

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

void LightViewerContext::draw_ui() {
  ImGui::Begin(context_name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBringToFrontOnFocus);

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

void LightViewerContext::reset_center() {
  canvas->camera_control->reset_center();
}

void LightViewerContext::use_orbit_camera_control(double distance, double theta, double phi) {
  canvas->camera_control.reset(new guik::OrbitCameraControlXY(distance, theta, phi));
}

void LightViewerContext::use_orbit_camera_control_xz(double distance, double theta, double phi) {
  canvas->camera_control.reset(new guik::OrbitCameraControlXZ(distance, theta, phi));
}

void LightViewerContext::use_topdown_camera_control(double distance, double theta) {
  canvas->camera_control.reset(new guik::TopDownCameraControl(distance, theta));
}

void LightViewerContext::use_arcball_camera_control(double distance, double theta, double phi) {
  Eigen::Quaternionf quat(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY()));

  canvas->camera_control.reset(new guik::ArcBallCameraControl(distance, quat));
}

Eigen::Vector4i LightViewerContext::pick_info(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_info(p, window);
}

float LightViewerContext::pick_depth(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_depth(p, 2);
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

}  // namespace guik