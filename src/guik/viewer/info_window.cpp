#include <guik/viewer/info_window.hpp>

#include <regex>
#include <chrono>
#include <fmt/format.h>
#include <glk/split.hpp>

namespace guik {

LightViewer::InfoWindow::InfoWindow() {
  show_cpu_info = show_gpu_info = true;
  async_cpu_info = std::async(std::launch::async, []() { return std::string("No CPU info"); });
  async_gpu_info = std::async(std::launch::async, []() { return std::string("No GPU info"); });
}

LightViewer::InfoWindow::~InfoWindow() {}

bool LightViewer::InfoWindow::draw_ui() {
  bool show_window = true;

  ImGui::Begin("info", &show_window, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
  ImGui::Checkbox("CPU", &show_cpu_info);
  ImGui::SameLine();
  ImGui::Checkbox("GPU", &show_gpu_info);
  ImGui::Separator();
  ImGui::Text("FPS:%.1f", ImGui::GetIO().Framerate);

  if (show_cpu_info) {
    ImGui::Separator();
    if (async_cpu_info.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      cpu_info = async_cpu_info.get();
      async_cpu_info = std::async(std::launch::async, [this]() { return get_cpu_info(); });
    }

    ImGui::Text("%s", cpu_info.c_str());
  }

  if (show_gpu_info) {
    ImGui::Separator();
    if (async_gpu_info.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      gpu_info = async_gpu_info.get();
      async_gpu_info = std::async(std::launch::async, [this]() { return get_gpu_info(); });
    }

    ImGui::Text("%s", gpu_info.c_str());
  }

  ImGui::End();

  return show_window;
}

std::string LightViewer::InfoWindow::get_cpu_info() const {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  FILE* fp = popen("vmstat 1 2 --unit M -a", "r");
  if (fp == nullptr) {
    return "Failed to retrieve CPU info";
  }

  std::stringstream sst;

  std::vector<char> buffer(1024);
  while (fgets(buffer.data(), buffer.size(), fp) != nullptr) {
    std::string line(buffer.data());

    std::regex pattern("([0-9]+)(\\s+[0-9]+){16}");
    std::smatch matched;

    if (!std::regex_search(line, matched, pattern)) {
      continue;
    }

    std::stringstream line_sst(line);
    std::vector<int> values(16);
    for (int i = 0; i < values.size(); i++) {
      line_sst >> values[i];
    }

    sst.str("");
    int mem_free = values[3];
    int mem_inact = values[4];
    int mem_active = values[5];
    double mem_percent = 100 * (mem_active) / static_cast<double>(mem_free + mem_inact + mem_active);
    int cpu_idle = values[14];

    sst << fmt::format("CPU {:02d} % Memory {:02d} % ({:5d} Mb / {:5d} Mb)", 100 - cpu_idle, static_cast<int>(mem_percent), mem_active, mem_free + mem_inact + mem_active);
  }

  pclose(fp);

  if (sst.str().empty()) {
    return "Failed to retrieve CPU info";
  }

  return sst.str();
}

std::string LightViewer::InfoWindow::get_gpu_info() const {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  FILE* fp = popen("nvidia-smi --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total --format=csv,noheader", "r");
  if (fp == nullptr) {
    return "Failed to retrieve GPU info";
  }

  std::stringstream sst;

  std::vector<char> buffer(1024);
  while (fgets(buffer.data(), buffer.size(), fp) != nullptr) {
    std::string line(glk::trim(buffer.data()));

    std::vector<std::string> tokens = glk::split(line, ',');
    if (tokens.size() != 6) {
      continue;
    }

    sst << fmt::format("{:3s} {:20s} : GPU {:5s}    Memory ({:10s} / {:10s})", tokens[0], tokens[1], tokens[2], tokens[4], tokens[5]);
  }

  if (sst.str().empty()) {
    return "Failed to retrieve GPU info";
  }

  return sst.str();
}
}  // namespace guik