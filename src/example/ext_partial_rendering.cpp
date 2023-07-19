#include <iostream>
#include <algorithm>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->disable_vsync();
  viewer->show_info_window();

  // 
  viewer->enable_partial_rendering();

  // Number of points
  int num_points = 8192 * 64;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;

  // Create a point cloud buffer and register it to the viewer
  const auto update_points = [&] {
    // Generate a point cloud buffer with random points
    std::vector<Eigen::Vector3f> points(num_points);
    std::generate(points.begin(), points.end(), [] { return Eigen::Vector3f::Random(); });
    cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);

    // Clear the partial rendering buffer
    viewer->clear_partial_rendering();

    // Clear drawables and register new cloud buffers
    viewer->clear_drawables();
    for (int x = -4; x <= 4; x += 4) {
      for (int y = -4; y <= 4; y += 4) {
        viewer->update_drawable(guik::anon(), cloud_buffer, guik::Rainbow().translate(x, y, 0).static_object());
      }
    }
  };

  update_points();

  int points_rendering_budget = 8192;
  viewer->register_ui_callback("ui_callback", [&] {
    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::DragInt("Num points", &num_points, 8192, 8192, std::numeric_limits<int>::max());
    ImGui::DragInt("Budget", &points_rendering_budget, 1024, 1024, std::numeric_limits<int>::max());

    if (ImGui::Button("Change num points")) {
      update_points();
    }

    // Enable partial rendering.
    // Only `points_rendering_budget` points are rendered every frame.
    // This greatly reduces the rendering cost and helps rendering a very large point cloud.
    if (ImGui::Button("Enable partial rendering")) {
      cloud_buffer->enable_partial_rendering(points_rendering_budget);
    }

    // Disable partial rendering.
    // All the points are rendered every frame.
    if (ImGui::Button("Disable partial rendering")) {
      cloud_buffer->disable_partial_rendering();
    }

    ImGui::End();
  });

  viewer->spin();

  return 0;
}