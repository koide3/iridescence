#include <iostream>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  for(int x = -5; x <= 5; x++) {
    for(int y = -5; y <= 5; y++) {
      Eigen::Vector4f color((x + 5) / 10.0f, (y + 5) / 10.0f, 1.0, 1.0f);
      Eigen::Affine3f model_matrix = Eigen::Translation3f(x, y, 0.0f) * Eigen::UniformScaling<float>(0.5f) * Eigen::Isometry3f::Identity();
      viewer->update_drawable("cube_" + std::to_string(x) + "_" + std::to_string(y), glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::FlatColor(color, model_matrix.matrix()));
    }
  }

  viewer->register_ui_callback("picking", [&]() {
    ImGuiIO& io = ImGui::GetIO();
    if(io.WantCaptureMouse) {
      return;
    }

    auto mouse_pos = ImGui::GetMousePos();
    if(ImGui::IsMouseClicked(1)) {
      // get the depth of the clicked position
      float depth = viewer->pick_depth(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
      if(depth >= 1.0) {
        return;
      }

      // 3D coordinate of the clicked position
      Eigen::Vector3f pos = viewer->unproject(Eigen::Vector2i(mouse_pos.x, mouse_pos.y), depth);

      // put a red cube on the clicked pos
      Eigen::Affine3f model_matrix = Eigen::Translation3f(pos.array().round()) * Eigen::UniformScaling<float>(0.75f) * Eigen::Isometry3f::Identity();
      viewer->update_drawable("clicked_pos", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::FlatColor(Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f), model_matrix.matrix()));
    }
  });

  viewer->spin();
  return 0;
}