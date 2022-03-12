#include <iostream>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  // enable information buffer to recognize picked boxes
  viewer->enable_info_buffer();

  for (int x = -5; x <= 5; x++) {
    for (int y = -5; y <= 5; y++) {
      Eigen::Vector4f color((x + 5) / 10.0f, (y + 5) / 10.0f, 1.0, 1.0f);
      auto transform = Eigen::Translation3f(x, y, 0.0f) * Eigen::UniformScaling<float>(0.5f);

      // draw boxes with unique IDs
      viewer->update_drawable(
        "cube_" + std::to_string(x) + "_" + std::to_string(y),
        glk::Primitives::cube(),
        guik::FlatColor(color, transform).add("info_values", Eigen::Vector4i(x, y, 0, 0)));
    }
  }

  viewer->register_ui_callback("picking", [&]() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
      return;
    }

    auto mouse_pos = ImGui::GetMousePos();
    if (ImGui::IsMouseClicked(1)) {
      Eigen::Vector4i info = viewer->pick_info(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
      std::cout << "info:" << info.transpose() << std::endl;

      if (info[3] == -1) {
        return;
      }

      // get the depth and 3d coordinate of the clicked position
      float depth = viewer->pick_depth(Eigen::Vector2i(mouse_pos.x, mouse_pos.y));
      Eigen::Vector3f pos = viewer->unproject(Eigen::Vector2i(mouse_pos.x, mouse_pos.y), depth);
      std::cout << "depth:" << depth << " pos:" << pos.transpose() << std::endl;

      // change the picked cube's color
      auto cube = viewer->find_drawable("cube_" + std::to_string(info[0]) + "_" + std::to_string(info[1]));
      if (cube.first) {
        cube.first->add("color_mode", guik::ColorMode::FLAT_COLOR).add("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
      }
    }
  });

  viewer->spin();
  return 0;
}