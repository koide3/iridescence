#ifndef GUIK_IMGUI_APPLICATION_HPP
#define GUIK_IMGUI_APPLICATION_HPP

#include <iostream>

#include <imgui.h>
#include <Eigen/Core>

struct GLFWwindow;

namespace guik {
class Application {
public:
  Application();
  virtual ~Application();

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version = "#version 330", bool background = false, const std::string& title = "screen");

  bool ok() const;

  void enable_vsync();
  void disable_vsync();

  void enable_docking();
  void disable_docking();

  Eigen::Vector2i window_size() const;
  virtual void show_window();
  virtual void hide_window();
  virtual void maximize_window();
  virtual void fullscreen_window();
  virtual void resize(const Eigen::Vector2i& size);

  void set_title(const std::string& title);

  Eigen::Vector2i framebuffer_size() const;
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size);

  void spin();
  bool spin_once();

  void begin_ui();
  void begin_gl();
  bool end();

  void close();
  bool closed();

  virtual void draw_ui();
  virtual void draw_gl();

protected:
  GLFWwindow* window;
};

}  // namespace guik

#endif