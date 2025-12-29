#include <iostream>
#include <glk/path.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/io/ply_io.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();
  viewer->disable_vsync();
  viewer->show_info_window();

  // Enable thepartial rendering mode. In this mode, only a part of point clouds are rendered in each frame.
  // Although this may cause flickering, it can drastically reduce the time for rendering static point clouds.
  // Note:
  // 1. In the partial rendering mode, every drawable must be marked as static or dynamic object (e.g., guik::FlatOrange().static_object()).
  // 2. For static point clouds, enable partial rendering by calling enable_partial_rendering() with the points rendering budget.
  const double clear_thresh = 1e-6;
  viewer->enable_partial_rendering(clear_thresh);

  auto ply = glk::load_ply(glk::get_data_path() + "/models/bunny.ply");
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);

  // Set the number of points to be rendered in each frame.
  int points_rendering_budget = 512;

  viewer->register_ui_callback("ui_callback", [&] {
    ImGui::DragInt("points_rendering_budget", &points_rendering_budget, 1, 1, 8192 * 5);
    if (ImGui::Button("Enable Partial Rendering")) {
      // Enable partial rendering with the specified budget.
      cloud_buffer->enable_partial_rendering(points_rendering_budget);
    }
  });

  // Draw many bunnies as high rendering load objects.
  for (int i = -5; i <= 5; i++) {
    for (int j = -5; j <= 5; j++) {
      for (int k = 0; k < 5; k++) {
        auto setting = guik::Rainbow()
                         // Setting transformation
                         .rotate(M_PI_2, Eigen::Vector3f::UnitX())
                         .scale(5.0f)
                         .translate(i, j, k)
                         // Mark as static object
                         .static_object();

        viewer->update_drawable("bunny_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k), cloud_buffer, setting);
      }
    }
  }

  double t = 0.0f;
  while (t += ImGui::GetIO().DeltaTime, viewer->spin_once()) {
    auto setting = guik::FlatBlue()
                     // Setting transformation
                     .translate(std::cos(t) * 10.0f, std::sin(t) * 10.0f, 0.0f)
                     .scale(1.0f, 1.0f, 20.0f)
                     // Mark as dynamic object
                     .dynamic_object();

    viewer->update_cube("cube", setting);
  }

  return 0;
}