#include <guik/imgui_application.hpp>

#include <iostream>
#include <unordered_map>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include <ImGuizmo.h>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <glk/console_colors.hpp>

namespace guik {

using namespace glk::console;

Application::Application() : window(nullptr) {}

Application ::~Application() {
  if (!window) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

// dirty implementation
std::unordered_map<GLFWwindow*, Application*> appmap;
void fb_size_callback(GLFWwindow* window, int width, int height) {
  appmap[window]->framebuffer_size_callback(Eigen::Vector2i(width, height));
}

bool Application::init(const Eigen::Vector2i& size, const char* glsl_version, bool background, const std::string& title) {
  glfwSetErrorCallback([](int err, const char* desc) { std::cerr << bold_red << "glfw error " << err << ": " << desc << reset << std::endl; });
  if (!glfwInit()) {
    std::cerr << bold_red << "failed to initialize GLFW" << reset << std::endl;
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  if (background) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  }

  window = glfwCreateWindow(size[0], size[1], title.c_str(), nullptr, nullptr);
  if (window == nullptr) {
    return false;
  }
  appmap[window] = this;

  glfwSetFramebufferSizeCallback(window, fb_size_callback);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (gl3wInit()) {
    std::cerr << bold_red << "failed to initialize GL3W" << reset << std::endl;
    return false;
  }

  // setup imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return true;
}

void Application::enable_vsync() {
  glfwSwapInterval(1);
}

void Application::disable_vsync() {
  glfwSwapInterval(0);
}

Eigen::Vector2i Application::framebuffer_size() const {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  return Eigen::Vector2i(width, height);
}

void Application::framebuffer_size_callback(const Eigen::Vector2i& size) {
  std::cout << "FB:" << size.transpose() << std::endl;
}

void Application::enable_docking() {
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void Application::disable_docking() {
  ImGui::GetIO().ConfigFlags ^= ImGuiConfigFlags_DockingEnable;
}

bool Application::ok() const {
  return !glfwWindowShouldClose(window);
}

Eigen::Vector2i Application::window_size() const {
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  return Eigen::Vector2i(width, height);
}

void Application::show_window() {
  glfwShowWindow(window);
}

void Application::hide_window() {
  glfwHideWindow(window);
}

void Application::maximize_window() {
  glfwMaximizeWindow(window);
}

void Application::fullscreen_window() {
  GLFWmonitor* monitor = glfwGetPrimaryMonitor();
  const GLFWvidmode* mode = glfwGetVideoMode(monitor);
  glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
}

void Application::resize(const Eigen::Vector2i& size) {
  glfwSetWindowSize(window, size[0], size[1]);
}

void Application::set_title(const std::string& title) {
  glfwSetWindowTitle(window, title.c_str());
}

void Application::spin() {
  glfwSetWindowShouldClose(window, GLFW_FALSE);
  while (spin_once()) {
  }
}

bool Application::spin_once() {
  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

  draw_ui();

  ImGui::Render();

  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.27f, 0.27f, 0.27f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  draw_gl();

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window);

  return !glfwWindowShouldClose(window);
}

void Application::begin_ui() {
  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
}

void Application::begin_gl() {
  ImGui::Render();

  int display_w, display_h;
  glfwGetFramebufferSize(window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(0.27f, 0.27f, 0.27f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

bool Application::end() {
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window);

  return !glfwWindowShouldClose(window);
}

void Application::close() {
  glfwSetWindowShouldClose(window, 1);
}

bool Application::closed() {
  return glfwWindowShouldClose(window);
}

void Application::draw_ui() {
  ImGui::ShowDemoWindow();
}

void Application::draw_gl() {}

}  // namespace guik
