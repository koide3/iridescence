#include <guik/viewer/light_viewer.hpp>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <regex>
#include <chrono>
#include <future>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <glk/io/png_io.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/console_colors.hpp>
#include <guik/viewer/viewer_ui.hpp>
#include <guik/viewer/info_window.hpp>

namespace guik {

std::shared_ptr<LightViewer> LightViewer::inst;

LightViewer::LightViewer() : Application(), LightViewerContext("main"), max_texts_size(32) {}

LightViewer::~LightViewer() {}

bool LightViewer::init(const Eigen::Vector2i& size, const char* glsl_version, bool background) {
  Application::init(size, glsl_version, background);
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

  // To allow removing a callback from a callback call, avoid directly iterating over ui_callbacks
  std::vector<const std::function<void()>*> callbacks;
  for (const auto& callback : ui_callbacks) {
    callbacks.emplace_back(&callback.second);
  }
  for (const auto& callback : callbacks) {
    (*callback)();
  }

  // viewer UI
  if(viewer_ui) {
    if(!viewer_ui->draw_ui()) {
      viewer_ui.reset();
    }
  } else if(ImGui::GetIO().KeyCtrl && ImGui::GetIO().KeysDown[GLFW_KEY_M]) {
    viewer_ui.reset(new ViewerUI(this));
  }

  // point scale
  bool decrease_point_size = ImGui::GetIO().KeysDown[GLFW_KEY_MINUS];
  bool increase_point_size = ImGui::GetIO().KeyShift && ImGui::GetIO().KeysDown[GLFW_KEY_SEMICOLON];
  if(decrease_point_size || increase_point_size) {
    auto point_size = global_shader_setting.get<float>("point_size");
    if(!point_size) {
      point_size = 10.0f;
    }

    if(decrease_point_size) {
      *point_size = point_size.get() - ImGui::GetIO().DeltaTime * 10.0f;
    } else {
      *point_size = point_size.get() + ImGui::GetIO().DeltaTime * 10.0f;
    }

    *point_size = std::max(0.1f, std::min(1e6f, point_size.get()));

    global_shader_setting.add("point_size", *point_size);
  }

  // screen shot
  if(ImGui::GetIO().KeysDown[GLFW_KEY_J]) {
    invoke_after_rendering([this] {
      auto bytes = canvas->frame_buffer->color().read_pixels<unsigned char>(GL_RGBA, GL_UNSIGNED_BYTE);
      std::vector<unsigned char> flipped(bytes.size(), 255);

      Eigen::Vector2i size = canvas->frame_buffer->color().size();
      for(int y = 0; y < size[1]; y++) {
        int y_ = size[1] - y - 1;
        for(int x = 0; x < size[0]; x++) {
          for(int k = 0; k < 3; k++) {
            flipped[(y_ * size[0] + x) * 4 + k] = bytes[(y * size[0] + x) * 4 + k];
          }
        }
      }

      double time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() / 1e9;
      std::string filename = (boost::format("/tmp/ss_%.6f.png") % time).str();
      if(glk::save_png(filename, canvas->size[0], canvas->size[1], flipped)) {
        std::cout << "screen shot saved:" << filename << std::endl;
      } else {
        std::cout << "failed to save screen shot" << std::endl;
      }
    });
  }

  if(info_window) {
    if(!info_window->draw_ui()) {
      info_window.reset();
    }
  }

  // texts
  std::vector<std::string> texts_;
  {
    std::lock_guard<std::mutex> texts_lock(texts_mutex);
    std::vector<std::string>(texts.begin(), texts.end()).swap(texts_);
  }

  if(!texts_.empty()) {
    ImGui::Begin("texts", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    for(int i = std::max<int>(0, texts_.size() - max_texts_size); i < texts_.size(); i++) {
      const auto& text = texts_[i];
      ImGui::Text("%s", text.c_str());
    }

    if(ImGui::Button("clear")) {
      clear_text();
    }
    ImGui::End();
  }

  // images
  if(!images.empty()) {
    ImGui::Begin("images", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    images_in_rendering.clear();
    std::unordered_map<std::string, std::vector<std::string>> groups;
    for(const auto& image: images) {
      const size_t separator_loc = image.first.find_first_of('/');
      const std::string group = separator_loc == std::string::npos ? "default" : image.first.substr(0, separator_loc);
      groups[group].push_back(image.first);
      images_in_rendering.emplace_back(image.second.second);
    }

    const bool grouping = groups.size() > 1;

    if(grouping) {
      ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
      ImGui::BeginTabBar("imagetab", tab_bar_flags);
    }

    for(auto& group: groups) {
      if(grouping) {
        if(!ImGui::BeginTabItem(group.first.c_str())) {
          continue;
        }
      }

      const auto& group_name = group.first;
      auto& image_names = group.second;
      std::sort(image_names.begin(), image_names.end());

      for(const auto& name: image_names) {
        const auto& image = images[name];
        const double scale = image.first;
        const auto& texture = image.second;

        Eigen::Vector2i size = (texture->size().cast<double>() * scale).cast<int>();

        ImGui::Text("%s", name.c_str());
        ImGui::Image(reinterpret_cast<void*>(texture->id()), ImVec2(size[0], size[1]), ImVec2(0, 0), ImVec2(1, 1));
      }

      if(grouping) {
        ImGui::EndTabItem();
      }
    }

    if(grouping) {
      ImGui::EndTabBar();
    }

    ImGui::End();
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

  std::unique_lock<std::mutex> lock(post_render_invoke_requests_mutex);
  while(!post_render_invoke_requests.empty()) {
    post_render_invoke_requests.front()();
    post_render_invoke_requests.pop_front();
  }
  lock.unlock();
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
  std::vector<std::string> texts;
  boost::split(texts, text, boost::is_any_of("\n"));

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
  if(found != images.end()) {
    images.erase(found);
  }
}

void LightViewer::update_image(const std::string& name, const std::shared_ptr<glk::Texture>& image, double scale) {
  if (!image) {
    remove_image(name);
    return;
  }

  if(scale < 0.0) {
    double scale_x = 640.0 / image->size()[0];
    double scale_y = 480.0 / image->size()[1];
    scale = std::min(scale_x, scale_y);
  }

  images[name] = std::make_pair(scale, image);
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

bool LightViewer::toggle_spin_once() {
  bool stop = false;

  register_ui_callback("kill_switch", [&]() { ImGui::Checkbox("break", &stop); });

  do {
    if(!spin_once()) {
      return false;
    }
  } while (stop);

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

void LightViewer::invoke_after_rendering(const std::function<void()>& func) {
  std::lock_guard<std::mutex> lock(post_render_invoke_requests_mutex);
  post_render_invoke_requests.push_back(func);
}

std::shared_ptr<LightViewerContext> LightViewer::sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size) {
  using namespace glk::console;

  auto found = sub_contexts.find(context_name);
  if(found == sub_contexts.end()) {
    Eigen::Vector2i init_canvas_size = canvas_size;
    if(canvas_size[0] <= 0 || canvas_size[1] <= 0) {
      init_canvas_size = Eigen::Vector2i(512, 512);
    }

    std::shared_ptr<LightViewerContext> context(new LightViewerContext(context_name));
    if(!context->init_canvas(init_canvas_size)) {
      std::cerr << bold_red << "error: failed to create sub viewer context!!" << reset << std::endl;
      return nullptr;
    }

    sub_contexts[context_name] = context;
    return context;
  }

  if((canvas_size.array() > 0).all() && found->second->canvas_size() != canvas_size) {
    if(!found->second->init_canvas(canvas_size)) {
      std::cerr << bold_red << "error: failed to resize the canvas of " << context_name << "!!" << reset << std::endl;
      close();
    }
  }

  return found->second;
}

std::vector<unsigned char> LightViewer::read_color_buffer() {
  auto bytes = canvas->frame_buffer->color().read_pixels<unsigned char>(GL_RGBA, GL_UNSIGNED_BYTE);
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

std::vector<float> LightViewer::read_depth_buffer(bool real_scale) {
  auto floats = canvas->frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);
  std::vector<float> flipped(floats.size());

  Eigen::Vector2i size = canvas->frame_buffer->color().size();
  for (int y = 0; y < size[1]; y++) {
    int y_ = size[1] - y - 1;
    for (int x = 0; x < size[0]; x++) {
        flipped[y_ * size[0] + x] = floats[y * size[0] + x];
    }
  }

  if (real_scale) {
    const Eigen::Vector2f depth_range = canvas->camera_control->depth_range();
    const float near = depth_range[0];
    const float far = depth_range[1];
    for (auto& depth : flipped) {
      depth = 2.0 * near * far / (far + near - depth * (far - near));
    }
  }

  return flipped;
}

}  // namespace guik