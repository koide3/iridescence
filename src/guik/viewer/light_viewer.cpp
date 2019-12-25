#include <guik/viewer/light_viewer.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/primitives/primitives.hpp>

namespace guik {

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
    drawable->draw(*canvas->shader);
  }

  canvas->unbind();
  canvas->render_to_screen();
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

}  // namespace guik