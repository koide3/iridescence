#ifndef GUIK_IMGUI_APPLICATION_HPP
#define GUIK_IMGUI_APPLICATION_HPP

#include <iostream>

#include <imgui.h>
#include <Eigen/Core>

class GLFWwindow;

namespace guik {
class Application {
public:
  Application();
  virtual ~Application();

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version = "#version 330");

  Eigen::Vector2i framebuffer_size();
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size);

  void spin();
  bool spin_once();

  void begin_ui();
  void begin_gl();
  bool end();

  void close();

  virtual void draw_ui();
  virtual void draw_gl();

protected:
  GLFWwindow* window;
};

}  // namespace guik

#endif