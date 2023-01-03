#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

class PointCloudEditor {
public:
  PointCloudEditor() {
    auto viewer = guik::LightViewer::instance();

    cube_matrix.reset(new guik::ModelControl("cube_matrix"));

    viewer->register_ui_callback("ui", [this] { ui_callback(); });
    viewer->spin();
  }

private:
  void ui_callback() {
    auto viewer = guik::LightViewer::instance();

    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Load points    
    if (ImGui::Button("Load point cloud")) {
      for (const auto& filename : pfd::open_file("Select a PLY file").result()) {
        // Load points from PLY and add them to the point list
        auto ply = glk::load_ply(filename);
        if (ply) {
          points.insert(points.end(), ply->vertices.begin(), ply->vertices.end());
        }
      }

      // Show points on the viewer
      if (!points.empty()) {
        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
        viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      }
    }

    // Save points
    if (ImGui::Button("Save point cloud")) {
      auto path = pfd::save_file("Select a destination path to save PLY").result();
      if (!path.empty() && !points.empty()) {
        glk::save_ply_binary(path, points.data(), points.size());
      }
    }

    // Show the cube representing the filtering area
    ImGui::Separator();
    cube_matrix->draw_gizmo_ui();
    cube_matrix->draw_gizmo();
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::FlatColor({1.0f, 0.5f, 0.0f, 0.5f}, cube_matrix->model_matrix()).make_transparent());

    // Find points in the filtering area
    if (ImGui::Button("Select points")) {
      // The inverse of the cube model matrix transforms world points in the cube coordinate system
      Eigen::Matrix4f inv_cube_matrix = cube_matrix->model_matrix().inverse();

      selected_points.clear();
      neg_selected_points.clear();
      for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f pt = (Eigen::Affine3f(inv_cube_matrix) * points[i]);
        // Points in [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)] are inside of the cube
        if ((pt.array() > Eigen::Array3f::Constant(-0.5f)).all() && (pt.array() < Eigen::Array3f::Constant(0.5f)).all()) {
          selected_points.emplace_back(i);
        } else {
          neg_selected_points.emplace_back(i);
        }
      }

      // Show the selected points with large orange points
      viewer->update_drawable("selected", std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, selected_points), guik::FlatOrange().set_point_scale(2.0f));
    }

    // Remove the selected points from the point list
    if (ImGui::Button("Remove points")) {
      // Leave only "un"selected points
      std::vector<Eigen::Vector3f> filtered;
      std::transform(neg_selected_points.begin(), neg_selected_points.end(), std::back_inserter(filtered), [&](const auto i) { return points[i]; });
      points = std::move(filtered);

      selected_points.clear();
      neg_selected_points.clear();

      cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
      viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      viewer->remove_drawable("selected");
    }

    ImGui::End();
  }

private:
  std::unique_ptr<guik::ModelControl> cube_matrix;      // Model matrix of the cube representing the filtering area

  std::vector<Eigen::Vector3f> points;                  // Point cloud
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;  // CloudBuffer of points
  std::vector<unsigned int> selected_points;            // Selected points
  std::vector<unsigned int> neg_selected_points;        // Negative of selected points
};

int main(int argc, char** argv) {
  PointCloudEditor editor;
  return 0;
}