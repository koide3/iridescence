#include <guik/viewer/light_viewer.hpp>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <regex>
#include <chrono>
#include <future>
#include <boost/format.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/viewer_ui.hpp>
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
  // if(!LightViewerContext::init_canvas(size)) {
  //  std::cerr << "error: failed to resize the canvas!!" << std::endl;
  //  close();
  // }
  LightViewerContext::set_size(size);
}

void LightViewer::draw_ui() {
  std::unique_lock<std::mutex> lock(invoke_requests_mutex);
  while(!invoke_requests.empty()) {
    invoke_requests.front()();
    invoke_requests.pop_front();
  }
  lock.unlock();

  if(viewer_ui) {
    if(!viewer_ui->draw_ui()) {
      viewer_ui.reset();
    }
  } else if(ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeysDown[GLFW_KEY_M]) {
    viewer_ui.reset(new ViewerUI(this));
  }

  bool decrease_point_size = ImGui::GetIO().KeysDown[GLFW_KEY_MINUS];
  bool increase_point_size = ImGui::GetIO().KeyShift && ImGui::GetIO().KeysDown[GLFW_KEY_SEMICOLON];
  if(decrease_point_size || increase_point_size) {
    auto point_size = global_shader_setting.get<float>("point_size");
    if(!point_size) {
      point_size = 10.0f;
    }

    if(decrease_point_size) {
      *point_size = point_size.get() * 0.9f;
    } else {
      *point_size = point_size.get() * 1.2f;
    }

    *point_size = std::max(0.1f, std::min(1e6f, point_size.get()));

    global_shader_setting.add("point_size", *point_size);
  }

  if(info_window) {
    if(!info_window->draw_ui()) {
      info_window.reset();
    }
  }

  std::vector<std::string> texts_;
  {
    std::lock_guard<std::mutex> texts_lock(texts_mutex);
    std::vector<std::string>(texts.begin(), texts.end()).swap(texts_);
  }

  if(!texts.empty()) {
    ImGui::Begin("texts", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground);
    for(int i = std::max<int>(0, texts_.size() - 32); i < texts_.size(); i++) {
      const auto& text = texts_[i];
      ImGui::Text("%s", text.c_str());
    }

    if(ImGui::Button("clear")) {
      clear_text();
    }
    ImGui::End();
  }

  for(const auto& callback : ui_callbacks) {
    callback.second();
  }

  // mouse control
  if(!ImGui::GetIO().WantCaptureMouse) {
    canvas->mouse_control();
  }

  for(const auto& context : sub_contexts) {
    context.second->draw_gl();
    context.second->draw_ui();
  }
}

void LightViewer::draw_gl() {
  LightViewerContext::draw_gl();
  canvas->render_to_screen();
}

void LightViewer::clear_text() {
  std::lock_guard<std::mutex> lock(texts_mutex);
  texts.clear();
}

void LightViewer::append_text(const std::string& text) {
  std::lock_guard<std::mutex> lock(texts_mutex);
  texts.push_back(text);
}

void LightViewer::clear() {
  invoke_requests_mutex.lock();
  invoke_requests.clear();
  invoke_requests_mutex.unlock();
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

void LightViewer::show_viewer_ui() {
  if(viewer_ui == nullptr) {
    viewer_ui.reset(new ViewerUI(this));
  }
}

void LightViewer::show_info_window() {
  if(info_window == nullptr) {
    info_window.reset(new InfoWindow());
  }
}

void LightViewer::invoke(const std::function<void()>& func) {
  std::lock_guard<std::mutex> lock(invoke_requests_mutex);
  invoke_requests.push_back(func);
}

std::shared_ptr<LightViewerContext> LightViewer::sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size_) {
  Eigen::Vector2i canvas_size = canvas_size_;
  if(canvas_size[0] <= 0 || canvas_size[1] <= 1) {
    canvas_size = Eigen::Vector2i(512, 512);
  }

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