#include <memory>
#include <imgui.h>

#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glk/glsl_shader.hpp>

#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/model_control.hpp>
#include <guik/imgui_application.hpp>

#include <ros/package.h>

class GLTestField : public guik::Application {
public:
  GLTestField() : Application() {}

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version) override {
    Application::init(size, glsl_version);

    std::string data_directory = ros::package::getPath("gl_test_field") + "/data";
    main_canvas.reset(new guik::GLCanvas(data_directory, size));
    if (!main_canvas->ready()) {
      close();
      return false;
    }

    primitive_type = 0;
    model_control.reset(new guik::ModelControl("cube"));

    return true;
  }

  virtual void draw_ui() override {
    // main menu
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Quit")) {
          close();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }

    {
      const char* items[] = {"ICOSAHEDRON", "SPHERE", "CUBE", "COORDINATE_SYSTEM", "BUNNY"};
      ImGui::Begin("primitive", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Combo("type", &primitive_type, items, IM_ARRAYSIZE(items));
      ImGui::End();
    }

    // render textures
    {
      ImGui::Begin("textures", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Text("depth");
      ImGui::Image((void*)(intptr_t)main_canvas->frame_buffer->depth().id(), ImVec2(1920 / 4, 1080 / 4), ImVec2(0, 1), ImVec2(1, 0));
      ImGui::Text("material color");
      ImGui::Image((void*)(intptr_t)main_canvas->frame_buffer->color(1).id(), ImVec2(1920 / 4, 1080 / 4), ImVec2(0, 1), ImVec2(1, 0));
      ImGui::Text("normal");
      ImGui::Image((void*)(intptr_t)main_canvas->frame_buffer->color(2).id(), ImVec2(1920 / 4, 1080 / 4), ImVec2(0, 1), ImVec2(1, 0));
      ImGui::End();
    }

    model_control->draw_ui();

    // mouse control
    if (!ImGui::GetIO().WantCaptureMouse) {
      main_canvas->mouse_control();
    }
  }

  virtual void draw_gl() override {
    double time = ImGui::GetTime();
    main_canvas->bind();

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // light setting
    Eigen::Vector3f light_pos(std::cos(time) * 5.0f, std::sin(time) * 5.0f, 0.5f);
    main_canvas->shader->set_uniform("light_pos", light_pos);
    main_canvas->shader->set_uniform("light_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
    main_canvas->shader->set_uniform("ambient_color", Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.0f));

    main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f));
    main_canvas->shader->set_uniform("material_emission", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));

    main_canvas->shader->set_uniform("model_matrix", (Eigen::Isometry3f::Identity() * Eigen::Translation3f(light_pos) * Eigen::UniformScaling<float>(0.1f)).matrix());
    glk::Primitives::instance()->primitive(glk::Primitives::SPHERE).draw(*main_canvas->shader);

    // draw primitive
    main_canvas->shader->set_uniform("material_shininess", 512.0f);
    main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
    main_canvas->shader->set_uniform("material_emission", Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f));

    main_canvas->shader->set_uniform("model_matrix", model_control->model_matrix());
    glk::Primitives::instance()->primitive(static_cast<glk::Primitives::PrimitiveType>(primitive_type)).draw(*main_canvas->shader);

    main_canvas->unbind();
    main_canvas->render_to_screen();
  }

private:
  int primitive_type;
  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ModelControl> model_control;
};

int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new GLTestField());

  if (!app->init(Eigen::Vector2i(1920, 1080), "#version 130")) {
    return 1;
  }

  app->spin();

  return 0;
}
