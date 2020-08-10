#include <guik/viewer/light_viewer.hpp>

#include <regex>
#include <chrono>
#include <future>
#include <boost/format.hpp>
#include <boost/process.hpp>
#include <boost/algorithm/string.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>

namespace guik {

class LightViewer::InfoWindow {
public:
  InfoWindow() {
    show_cpu_info = show_gpu_info = true;
    async_cpu_info = std::async(std::launch::async, [](){ return std::string("No CPU info"); });
    async_gpu_info = std::async(std::launch::async, [](){ return std::string("No GPU info"); });
  }
  ~InfoWindow() {

  }

  bool draw_ui() {
    bool show_window = true;

    ImGui::Begin("info", &show_window, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
    ImGui::Checkbox("CPU", &show_cpu_info);
    ImGui::SameLine();
    ImGui::Checkbox("GPU", &show_gpu_info);
    ImGui::Separator();
    ImGui::Text("FPS:%.1f", ImGui::GetIO().Framerate);

    if(show_cpu_info) {
      ImGui::Separator();
      if(async_cpu_info.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        cpu_info = async_cpu_info.get();
        async_cpu_info = std::async(std::launch::async, [this]() { return get_cpu_info(); });
      }

      ImGui::Text(cpu_info.c_str());
    }

    if(show_gpu_info) {
      ImGui::Separator();
      if(async_gpu_info.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        gpu_info = async_gpu_info.get();
        async_gpu_info = std::async(std::launch::async, [this]() { return get_gpu_info(); });
      }

      ImGui::Text(gpu_info.c_str());
    }

    ImGui::End();

    return show_window;
  }

private:
  std::string get_cpu_info() const {
    boost::process::ipstream pipe_stream;
    boost::process::child c("vmstat 1 2 --unit M -a", boost::process::std_out > pipe_stream);

    std::stringstream sst;

    std::string line;
    while(pipe_stream && std::getline(pipe_stream, line) && !line.empty()) {
      std::regex pattern("([0-9]+)(\\s+[0-9]+){16}");
      std::smatch matched;

      if(!std::regex_search(line, matched, pattern)) {
        continue;
      }

      std::stringstream line_sst(line);
      std::vector<int> values(16);
      for(int i=0; i<values.size(); i++) {
        line_sst >> values[i];
      }

      sst.str("");
      int mem_free = values[3];
      int mem_inact = values[4];
      int mem_active = values[5];
      double mem_percent = 100 * mem_active / static_cast<double>(mem_free + mem_inact + mem_active);
      int cpu_idle = values[14];

      sst << boost::format("CPU %02d %%  Memory %02d %% (%5d Mb / %5d Mb)") % (100 - cpu_idle) % static_cast<int>(mem_percent) % mem_active % (mem_free + mem_inact + mem_active);
    }
    c.wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(sst.str().empty()) {
      return "Failed to retrieve CPU info";
    }

    return sst.str();
  }

  std::string get_gpu_info() const {
    boost::process::ipstream pipe_stream;
    boost::process::child c("nvidia-smi --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total --format=csv,noheader", boost::process::std_out > pipe_stream);

    std::stringstream sst;

    std::string line;
    while(pipe_stream && std::getline(pipe_stream, line) && !line.empty()) {
      std::vector<std::string> tokens;
      boost::split(tokens, line, boost::is_any_of(","));

      if(tokens.size() != 6) {
        continue;
      }

      sst << boost::format("%5s %20s : GPU %5s    Memory %5s (%10s / %10s)\n") % tokens[0] % tokens[1] % tokens[2] % tokens[3] % tokens[4] % tokens[5];
    }
    c.wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(sst.str().empty()) {
      return "Failed to retrieve GPU info";
    }

    return sst.str();
  }

private:
  bool show_cpu_info;
  std::string cpu_info;
  std::future<std::string> async_cpu_info;

  bool show_gpu_info;
  std::string gpu_info;
  std::future<std::string> async_gpu_info;
};

LightViewer* LightViewer::inst = nullptr;

LightViewer::LightViewer() : Application() {}

LightViewer::~LightViewer() {}

bool LightViewer::init(const Eigen::Vector2i& size, const char* glsl_version) {
  Application::init(size, glsl_version);
  // GL::Renderer::setClearColor(0x474747_rgbf); // lightwave
  // GL::Renderer::setClearColor(0x3a3a3a_rgbf); // blender

  std::string data_directory = ros::package::getPath("gl_test_field") + "/data";
  canvas.reset(new guik::GLCanvas(data_directory, size));
  if(!canvas->ready()) {
    close();
    return false;
  }

  return true;
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
}

void LightViewer::draw_gl() {
  canvas->bind();

  canvas->shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  canvas->shader->set_uniform("color_mode", 1);
  canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.6f, 0.6f, 0.6f, 1.0f));
  glk::Primitives::instance()->primitive(glk::Primitives::GRID).draw(*canvas->shader);

  for(const auto& iter : drawables) {
    const auto& shader_setting = iter.second.first;
    shader_setting->set(*canvas->shader);

    const auto& drawable = iter.second.second;
    if(drawable) {
      drawable->draw(*canvas->shader);
    }
  }

  canvas->unbind();
  canvas->render_to_screen();
}

void LightViewer::lookat(const Eigen::Vector3f& pt) {
  canvas->camera_control->lookat(pt);
}

void LightViewer::clear_text() {
  texts.clear();
}

void LightViewer::append_text(const std::string& text) {
  texts.push_back(text);
}

void LightViewer::clear() {
  ui_callbacks.clear();
  drawables.clear();
}

void LightViewer::clear_drawables() {
  drawables.clear();
}

void LightViewer::clear_drawables(const std::function<bool(const std::string&)>& fn) {
  for(auto it = drawables.begin(); it != drawables.end(); ) {
    if(fn(it->first)) {
      it = drawables.erase(it);
    } else {
      it++;
    }
  }
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

void LightViewer::update_drawable(const std::string& name, const glk::Drawable::Ptr& drawable, const ShaderSetting& shader_setting) {
  drawables[name] = std::make_pair(std::make_shared<ShaderSetting>(shader_setting), drawable);
}


void LightViewer::show_info_window() {
  if(info_window == nullptr) {
    info_window.reset(new InfoWindow());
  }
}

ShaderSetting::Ptr LightViewer::shader_setting(const std::string& name) {
  auto found = drawables.find(name);
  if(found == drawables.end()) {
    return nullptr;
  }

  return found->second.first;
}

}  // namespace guik