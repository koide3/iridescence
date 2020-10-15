#include <guik/viewer/light_viewer_context.hpp>

#include <glk/primitives/primitives.hpp>

namespace guik {

LightViewerContext::LightViewerContext(const std::string& context_name) : context_name(context_name) {}

LightViewerContext::~LightViewerContext() {}

bool LightViewerContext::init_canvas(const Eigen::Vector2i& size) {
  canvas.reset(new guik::GLCanvas(size));
  if(!canvas->ready()) {
    return false;
  }

  global_shader_setting.add("model_matrix", Eigen::Matrix4f::Identity().eval());

  return true;
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
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.6f, 0.6f, 0.6f, 1.0f));
  glk::Primitives::instance()->primitive(glk::Primitives::GRID).draw(*canvas->shader);

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

void LightViewerContext::set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
  canvas->set_effect(effect);
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

std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr> LightViewerContext::find_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if(found != drawables.end()) {
    return found->second;
  }
  return std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>();
}

void LightViewerContext::remove_drawable(const std::string& name) {
  auto found = drawables.find(name);
  if(found != drawables.end()) {
    drawables.erase(found);
  }
}

void LightViewerContext::update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting) {
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

float LightViewerContext::pick_depth(const Eigen::Vector2i& p, int window) const {
  return canvas->pick_depth(p, 2);
}

Eigen::Vector3f LightViewerContext::unproject(const Eigen::Vector2i& p, float depth) const {
  return canvas->unproject(p, depth);
}

}  // namespace guik