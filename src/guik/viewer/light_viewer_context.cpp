#include <guik/viewer/light_viewer_context.hpp>

#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>

namespace guik {

LightViewerContext::LightViewerContext(const std::string& context_name) : context_name(context_name) {
  draw_xy_grid = true;
}

LightViewerContext::~LightViewerContext() {}

bool LightViewerContext::init_canvas(const Eigen::Vector2i& size) {
  canvas.reset(new guik::GLCanvas(size));
  if(!canvas->ready()) {
    return false;
  }

  global_shader_setting.add("model_matrix", Eigen::Matrix4f::Identity().eval());

  return true;
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

void LightViewerContext::draw_ui() {
  ImGui::Begin(context_name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  ImGuiWindowFlags flags = ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavFocus;
  ImGui::BeginChild("canvas", ImVec2(canvas->size[0], canvas->size[1]), false, flags);
  if(ImGui::IsWindowFocused()) {
    canvas->mouse_control();
  }

  ImGui::Image((void*)canvas->frame_buffer->color().id(), ImVec2(canvas->size[0], canvas->size[1]), ImVec2(0, 1), ImVec2(1, 0));
  ImGui::EndChild();

  ImGui::End();
}

void LightViewerContext::draw_gl() {
  canvas->bind();

  global_shader_setting.set(*canvas->shader);

  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
  canvas->shader->set_uniform("color_mode", 1);
  if(draw_xy_grid) {
    canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.6f, 0.6f, 0.6f, 1.0f));
    glk::Primitives::instance()->primitive(glk::Primitives::GRID).draw(*canvas->shader);
  }

  for(const auto& itr : drawables) {
    bool draw = true;
    for(const auto& filter : drawable_filters) {
      if(!filter.second(itr.first)) {
        draw = false;
        break;
      }
    }

    if(!draw) {
      continue;
    }

    const auto& shader_setting = itr.second.first;
    shader_setting->set(*canvas->shader);

    const auto& drawable = itr.second.second;
    if(drawable) {
      drawable->draw(*canvas->shader);
    }
  }

  canvas->unbind();
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

void LightViewerContext::enable_info_buffer() {
  canvas->enable_info_buffer();
}

void LightViewerContext::clear_drawables() {
  drawables.clear();
}

void LightViewerContext::clear_drawables(const std::function<bool(const std::string&)>& fn) {
  for(auto it = drawables.begin(); it != drawables.end();) {
    if(fn(it->first)) {
      it = drawables.erase(it);
    } else {
      it++;
    }
  }
}

std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr> LightViewerContext::find_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if(found != drawables.end()) {
    return found->second;
  }
  return std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>();
}

void LightViewerContext::remove_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if(found != drawables.end()) {
    drawables.erase(found);
  }
}

void LightViewerContext::update_drawable(const std::string& name, const glk::Drawable::ConstPtr& drawable, const ShaderSetting& shader_setting) {
  drawables[name] = std::make_pair(std::make_shared<ShaderSetting>(shader_setting), drawable);
}

void LightViewerContext::clear_drawable_filters() {
  drawable_filters.clear();
}

void LightViewerContext::add_drawable_filter(const std::string& filter_name, const std::function<bool(const std::string&)>& filter) {
  drawable_filters[filter_name] = filter;
}

void LightViewerContext::remove_drawable_filter(const std::string& filter_name) {
  auto found = drawable_filters.find(filter_name);
  if(found != drawable_filters.end()) {
    drawable_filters.erase(found);
  }
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

Eigen::Vector4i LightViewerContext::pick_info(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_info(p, window);
}

float LightViewerContext::pick_depth(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_depth(p, 2);
}

Eigen::Vector3f LightViewerContext::unproject(const Eigen::Vector2i& p, float depth) const {
  return canvas->unproject(p, depth);
}

}  // namespace guik