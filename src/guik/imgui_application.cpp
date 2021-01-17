#include <guik/imgui_application.hpp>

#include <iostream>
#include <unordered_map>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuizmo.h>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

namespace guik {

Application::Application() : window(nullptr) {}

Application ::~Application() {
  if (!window) {
    return;
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

// dirty implementation
std::unordered_map<GLFWwindow*, Application*> appmap;
void fb_size_callback(GLFWwindow* window, int width, int height) {
  appmap[window]->framebuffer_size_callback(Eigen::Vector2i(width, height));
}

bool Application::init(const Eigen::Vector2i& size, const char* glsl_version) {
  glfwSetErrorCallback([](int err, const char* desc) {
    std::cerr << "glfw error " << err << ": " << desc << std::endl;
  });
  if (!glfwInit()) {
    std::cerr << "failed to initialize GLFW" << std::endl;
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  window = glfwCreateWindow(size[0], size[1], "screen", nullptr, nullptr);
  if (window == nullptr) {
    return false;
  }
  appmap[window] = this;

  glfwSetFramebufferSizeCallback(window, fb_size_callback);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(0);

  if (gl3wInit()) {
    std::cerr << "failed to initialize GL3W" << std::endl;
    return false;
  }

  // setup imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return true;
}

Eigen::Vector2i Application::framebuffer_size() const {
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  return Eigen::Vector2i(width, height);
}

void Application::framebuffer_size_callback(const Eigen::Vector2i& size) {
  std::cout << "FB:" << size.transpose() << std::endl;
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

void Application::full_screen() {
  glfwMaximizeWindow(window);
}

void Application::resize(const Eigen::Vector2i& size) {
  glfwSetWindowSize(window, size[0], size[1]);
}

void Application::spin() {
  while(spin_once()) {
  }
}

bool Application::spin_once() {
  if(!glfwGetWindowAttrib(window, GLFW_VISIBLE)) {
    return !glfwWindowShouldClose(window);
  }

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
  glClearColor(0.6f, 0.6f, 0.6f, 1.0f);
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

void Application::draw_ui() { ImGui::ShowDemoWindow(); }

void Application::draw_gl() {}

}  // namespace guik
