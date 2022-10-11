#include <iostream>

#include <portable-file-dialogs.h>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  const auto paths = pfd::open_file("Select PLY file", "", std::vector<std::string>{"PLY files", "*.ply"}).result();
  if (paths.empty()) {
    return 0;
  }

  const auto ply = glk::load_ply(paths[0]);
  if (!ply) {
    std::cerr << "failed to open " << paths[0] << std::endl;
    return 0;
  }

  auto viewer = guik::LightViewer::instance();
  viewer->disable_vsync();
  viewer->show_info_window();
  viewer->enable_partial_rendering();

  Eigen::Affine3f model_matrix = Eigen::Affine3f::Identity();
  model_matrix.linear() = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(15.0f);
  model_matrix.translation() = Eigen::Vector3f(0.0f, -15.0f, 0.0f);

  viewer->update_drawable("bunny", glk::Primitives::bunny(), guik::Rainbow(model_matrix).add("dynamic_object", 0));

  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  viewer->update_drawable("cloud", cloud_buffer, guik::Rainbow().add("dynamic_object", 0));

  int points_rendering_budget = 8192 * 5;
  viewer->register_ui_callback("ui", [&] {
    ImGui::Begin("Partial rendering", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Points:%zu", ply->vertices.size());
    ImGui::DragInt("Budget", &points_rendering_budget, 1024, 1024, std::numeric_limits<int>::max());

    if (ImGui::Button("Enable")) {
      cloud_buffer->enable_partial_rendering(points_rendering_budget);
    }
    if (ImGui::Button("Disable")) {
      cloud_buffer->disable_partial_rendering();
    }

    ImGui::End();
  });

  double t = 0.0;
  while (viewer->spin_once()) {
    t += 1e-3;
    Eigen::Affine3f model_matrix = Eigen::Affine3f::Identity();
    model_matrix.linear() = Eigen::AngleAxisf(t * 0.5, Eigen::Vector3f::Ones().normalized()).toRotationMatrix() * Eigen::UniformScaling<float>(15.0f);
    viewer->update_drawable("coord", glk::Primitives::coordinate_system(), guik::VertexColor(model_matrix).add("dynamic_object", 255));
  }

  return 0;
}