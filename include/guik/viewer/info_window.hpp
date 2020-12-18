#ifndef GUIK_INFO_WINDOW_HPP
#define GUIK_INFO_WINDOW_HPP

#include <future>
#include <guik/viewer/light_viewer.hpp>

namespace guik {

class LightViewer::InfoWindow {
public:
  InfoWindow();
  ~InfoWindow();

  bool draw_ui();

private:
  std::string get_cpu_info() const;
  std::string get_gpu_info() const;

private:
  bool show_cpu_info;
  std::string cpu_info;
  std::future<std::string> async_cpu_info;

  bool show_gpu_info;
  std::string gpu_info;
  std::future<std::string> async_gpu_info;
};
}

#endif