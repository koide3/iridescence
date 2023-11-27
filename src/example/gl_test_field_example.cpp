#include <memory>
#include <imgui.h>
#include <ImGuizmo.h>

#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glk/glsl_shader.hpp>

#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/model_control.hpp>
#include <guik/imgui_application.hpp>

// Full application example
class GLTestField : public guik::Application {
public:
  GLTestField() : Application() {}

  virtual bool init(const Eigen::Vector2i& size, const char* glsl_version, bool background = false, const std::string& title = "screen") override {
    Application::init(size, glsl_version);

    main_canvas.reset(new guik::GLCanvas(size, "phong", 3));
    if (!main_canvas->ready()) {
      close();
      return false;
    }

    model_control.reset(new guik::ModelControl("cube"));

    return true;
  }

  /**
   * @brief callback to render ImGUI related things
   */
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
      const char* items[] = {"ICOSAHEDRON", "SPHERE", "CUBE", "CONE", "GRID", "COORDINATE_SYSTEM", "BUNNY"};
      ImGui::Begin("primitive", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Combo("type", &primitive_type_combo, items, IM_ARRAYSIZE(items));

      // because COORDINATE_SYSTEM falls through to SOLID_COORDINATE_SYSTEM in Primitives::create_primitive,
      //increment bunny toggle from gui (in items) by one if primitive type is 6 (COORDINATE_SYSTEM)
      primitive_type_render = (primitive_type_combo == 6) ? primitive_type_combo + 1 : primitive_type_combo;

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

    Eigen::Matrix4f view_matrix = main_canvas->camera_control->view_matrix();
    Eigen::Matrix4f projection_matrix = main_canvas->projection_control->projection_matrix();

    model_control->draw_ui();
    model_control->draw_gizmo(0, 0, 1920, 1080, view_matrix, projection_matrix);

    // mouse control
    if (!ImGui::GetIO().WantCaptureMouse && !ImGuizmo::IsUsing()) {
      main_canvas->mouse_control();
    }
  }

  /**
   * @brief callback to render OpenGL related things
   */
  virtual void draw_gl() override {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    main_canvas->bind();

    // light setting
    double time = ImGui::GetTime();
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
    glk::Primitives::primitive(static_cast<glk::Primitives::PrimitiveType>(primitive_type_render)).draw(*main_canvas->shader);

    main_canvas->unbind();
    main_canvas->render_to_screen();
  }

private:
  int primitive_type_combo = 0;
  int primitive_type_render = 0;
  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ModelControl> model_control;
};

int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new GLTestField());

  if (!app->init(Eigen::Vector2i(1920, 1080), "#version 330")) {
    return 1;
  }

  app->spin();

  return 0;
}
