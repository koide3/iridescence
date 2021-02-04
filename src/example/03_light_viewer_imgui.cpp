#include <imgui.h>

#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  // main menu example
  viewer->register_ui_callback("main_menu_ui", [&]() {
    ImGui::BeginMainMenuBar();
    if(ImGui::BeginMenu("File")) {
      if(ImGui::MenuItem("Quit")) {
        viewer->close();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  });

  // UI callback can be unregistered by giving null
  // viewer->register_ui_callback("main_menu_ui", nullptr);

  // control model matrix with guik::ModelControl
  guik::ModelControl model_control("model control window");
  viewer->register_ui_callback("model_control_ui", [&]() {
    ImGui::SetNextWindowPos(ImVec2(50, 50), ImGuiCond_FirstUseEver);
    model_control.draw_ui();
    // Note: gizmo must not be drawn twice on one rendering frame
    model_control.draw_gizmo(0, 0, viewer->canvas_size().x(), viewer->canvas_size().y(), viewer->view_matrix(), viewer->projection_matrix());
    viewer->update_drawable("coord", glk::Primitives::coordinate_system(), guik::VertexColor(model_control.model_matrix()));
  });

  // control model matrix with your own GUI
  Eigen::Affine3f my_transform = Eigen::Translation3f(1.0f, 1.0f, 1.0f) * Eigen::Affine3f::Identity();
  viewer->register_ui_callback("my_model_control_ui", [&]() {
    ImGui::SetNextWindowPos(ImVec2(50, 250), ImGuiCond_FirstUseEver);
    ImGui::Begin("my model control window", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if(ImGui::Button("+X")) {
      my_transform = Eigen::Translation3f(1.0f, 0.0f, 0.0f) * my_transform;
    }
    ImGui::SameLine();
    if(ImGui::Button("+Y")) {
      my_transform = Eigen::Translation3f(0.0f, 1.0f, 0.0f) * my_transform;
    }
    ImGui::SameLine();
    if(ImGui::Button("+Z")) {
      my_transform = Eigen::Translation3f(0.0f, 0.0f, 1.0f) * my_transform;
    }
    ImGui::End();

    viewer->update_drawable("my_coord", glk::Primitives::coordinate_system(), guik::VertexColor(my_transform));
  });

  // create sub viewer
  auto sub_viewer = viewer->sub_viewer("sub viewer", Eigen::Vector2i(512, 512));
  sub_viewer->set_pos(Eigen::Vector2i(50, 450), ImGuiCond_FirstUseEver);
  sub_viewer->update_drawable("icosahedron", glk::Primitives::icosahedron(), guik::Rainbow());

  // create another sub viewer with GLCanvas for low-level control
  guik::GLCanvas canvas(Eigen::Vector2i(512, 512));
  viewer->register_ui_callback("sub_gl_window_ui", [&]() {
    // rendering (this can be done outside of this callback)
    canvas.bind();
    guik::Rainbow().set(*canvas.shader);
    glk::Primitives::icosahedron()->draw(*canvas.shader);
    canvas.unbind();

    // show the image rendered on canvas's frame buffer
    ImGui::SetNextWindowPos(ImVec2(1300, 450), ImGuiCond_FirstUseEver);
    ImGui::Begin("your sub window", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGuiWindowFlags child_window_flags = ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavFocus;
    ImGui::BeginChild("canvas", ImVec2(512, 512), false, child_window_flags);

    if(ImGui::IsWindowFocused() && !ImGuizmo::IsUsing()) {
      canvas.mouse_control();
    }

    ImGui::Image((void*)canvas.frame_buffer->color().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));

    ImGui::EndChild();
    ImGui::End();
  });

  viewer->spin();

  return 0;
}
