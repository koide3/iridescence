#include <guik/viewer/light_viewer.hpp>

#include <regex>
#include <chrono>
#include <future>
#include <boost/format.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/info_window.hpp>

namespace guik {

LightViewer* LightViewer::inst = nullptr;

LightViewer::LightViewer() : Application(), LightViewerContext("main") {}

LightViewer::~LightViewer() {}

bool LightViewer::init(const Eigen::Vector2i& size, const char* glsl_version) {
  Application::init(size, glsl_version);
  // GL::Renderer::setClearColor(0x474747_rgbf); // lightwave
  // GL::Renderer::setClearColor(0x3a3a3a_rgbf); // blender

  if(!LightViewerContext::init_canvas(size)) {
    close();
    return false;
  }

  return true;
}

void LightViewer::framebuffer_size_callback(const Eigen::Vector2i& size) {
  if(!LightViewerContext::init_canvas(size)) {
    std::cerr << "error: failed to resize the canvas!!" << std::endl;
    close();
  }
}

void LightViewer::draw_ui() {
  if(info_window) {
    if(!info_window->draw_ui()) {
      info_window.reset();
    }
  }

  if(!texts.empty()) {
    ImGui::Begin("texts", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground);
    for(int i=std::max<int>(0, texts.size() - 32); i<texts.size(); i++) {
      const auto& text = texts[i];
      ImGui::Text(text.c_str());
    }

    if(ImGui::Button("clear")) {
      texts.clear();
    }
    ImGui::End();
  }

  for(const auto& callback: ui_callbacks) {
    callback.second();
  }

  // mouse control
  if(!ImGui::GetIO().WantCaptureMouse) {
    canvas->mouse_control();
  }

  for(const auto& context: sub_contexts) {
    context.second->draw_gl();
    context.second->draw_ui();
  }
}

void LightViewer::draw_gl() {
  LightViewerContext::draw_gl();
  canvas->render_to_screen();
}

void LightViewer::clear_text() {
  texts.clear();
}

void LightViewer::append_text(const std::string& text) {
  texts.push_back(text);
}

void LightViewer::clear() {
  ui_callbacks.clear();
  clear_drawables();
}

bool LightViewer::spin_until_click() {
  bool kill_switch = false;

  register_ui_callback("kill_switch", [&]() { kill_switch = ImGui::Button("break"); });

  while(!kill_switch) {
    if(!spin_once()) {
      return false;
    }
  }

  register_ui_callback("kill_switch", nullptr);

  return true;
}

void LightViewer::register_ui_callback(const std::string& name, const std::function<void()>& callback) {
  if(!callback) {
    ui_callbacks.erase(name);
    return;
  }

  ui_callbacks[name] = callback;
}

void LightViewer::show_info_window() {
  if(info_window == nullptr) {
    info_window.reset(new InfoWindow());
  }
}

std::shared_ptr<LightViewerContext> LightViewer::sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size) {
  auto found = sub_contexts.find(context_name);
  if(found == sub_contexts.end()) {
    std::shared_ptr<LightViewerContext> context(new LightViewerContext(context_name));
    if(!context->init_canvas(canvas_size)) {
      std::cerr << "error: failed to create sub viewer context!!" << std::endl;
      return nullptr;
    }

    sub_contexts[context_name] = context;
    return context;
  }

  if(found->second->canvas_size() != canvas_size) {
    if(!found->second->init_canvas(canvas_size)) {
      std::cerr << "error: failed to resize the canvas of " << context_name << "!!" << std::endl;
      close();
    }
  }

  return found->second;
}

}  // namespace guik