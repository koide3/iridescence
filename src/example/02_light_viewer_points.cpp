#include <iostream>
#include <portable-file-dialogs.h>

#include <glk/path.hpp>
#include <glk/colormap.hpp>
#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointnormals_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

// Point cloud with per-point attributes
struct PointCloud {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> normals;  // Empty if not available
  std::vector<Eigen::Vector4f> colors;   // RGBA in [0, 1]
  std::vector<float> intensities;        // Normalized to [0, 1]
  std::vector<float> cmaps;              // Arbitrary scalar values (here, distance from the center)
  Eigen::Vector3f center;                // Center of the bounding box
  float extent;                          // Longest edge length of the bounding box
};

// Load a PLY file and prepare per-point attributes. Attributes that are not in the file are synthesized.
std::shared_ptr<PointCloud> load_points(const std::string& path) {
  // glk::PLYData holds vertices, normals, colors, intensities, indices (faces), and generic properties
  const auto ply = glk::load_ply(path);
  if (!ply || ply->vertices.empty()) {
    return nullptr;
  }

  auto cloud = std::make_shared<PointCloud>();
  cloud->vertices = ply->vertices;
  const size_t num_points = cloud->vertices.size();

  Eigen::Vector3f min_pt = cloud->vertices[0];
  Eigen::Vector3f max_pt = cloud->vertices[0];
  for (const auto& v : cloud->vertices) {
    min_pt = min_pt.cwiseMin(v);
    max_pt = max_pt.cwiseMax(v);
  }
  cloud->center = (min_pt + max_pt) / 2.0f;
  cloud->extent = std::max((max_pt - min_pt).maxCoeff(), 1e-6f);

  // Normals: use the ones in the file, or estimate them from the faces if available
  if (ply->normals.size() == num_points) {
    cloud->normals = ply->normals;
  } else if (!ply->indices.empty()) {
    cloud->normals.assign(num_points, Eigen::Vector3f::Zero());
    for (size_t i = 0; i + 2 < ply->indices.size(); i += 3) {
      const int i0 = ply->indices[i], i1 = ply->indices[i + 1], i2 = ply->indices[i + 2];
      const Eigen::Vector3f n = (cloud->vertices[i1] - cloud->vertices[i0]).cross(cloud->vertices[i2] - cloud->vertices[i0]);
      cloud->normals[i0] += n;
      cloud->normals[i1] += n;
      cloud->normals[i2] += n;
    }
    for (auto& n : cloud->normals) {
      n = n.squaredNorm() > 0.0f ? n.normalized().eval() : Eigen::Vector3f::UnitZ().eval();
    }
  }

  // Colors: use the ones in the file, or encode the normals (or positions) as colors
  if (ply->colors.size() == num_points) {
    cloud->colors = ply->colors;
  } else {
    cloud->colors.resize(num_points);
    for (size_t i = 0; i < num_points; i++) {
      const Eigen::Vector3f rgb = cloud->normals.empty() ? ((cloud->vertices[i] - min_pt) / cloud->extent).eval() : (0.5f * cloud->normals[i] + Eigen::Vector3f::Constant(0.5f)).eval();
      cloud->colors[i] << rgb, 1.0f;
    }
  }

  // Intensities: use the ones in the file, or use the Y coordinates if they are not available or constant
  const auto normalize = [](std::vector<float>& values) {
    const auto minmax = std::minmax_element(values.begin(), values.end());
    const float min = *minmax.first;
    const float range = *minmax.second - *minmax.first;
    for (auto& value : values) {
      value = range > 1e-6f ? (value - min) / range : 0.0f;
    }
    return range > 1e-6f;
  };

  cloud->intensities = ply->intensities;
  if (cloud->intensities.size() != num_points || !normalize(cloud->intensities)) {
    cloud->intensities.resize(num_points);
    std::transform(cloud->vertices.begin(), cloud->vertices.end(), cloud->intensities.begin(), [](const Eigen::Vector3f& v) { return v.y(); });
    normalize(cloud->intensities);
  }

  // Colormap values: distance from the center (not normalized, the range is given to the shader)
  cloud->cmaps.resize(num_points);
  std::transform(cloud->vertices.begin(), cloud->vertices.end(), cloud->cmaps.begin(), [&](const Eigen::Vector3f& v) { return (v - cloud->center).norm(); });

  return cloud;
}

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  // The Stanford bunny in the data directory is selected by default
  std::string ply_path = glk::get_data_path() + "/models/bunny.ply";
  std::shared_ptr<PointCloud> cloud = load_points(ply_path);
  if (!cloud) {
    std::cerr << "failed to load " << ply_path << std::endl;
    return 1;
  }

  // Rendering settings
  const std::vector<const char*> color_modes = {"RAINBOW", "FLAT_COLOR", "VERTEX_COLOR (colors)", "VERTEX_COLOR (intensities)", "VERTEX_COLORMAP (cmaps)"};
  const std::vector<const char*> colormap_names = glk::colormap_names();
  int color_mode = 0;
  int colormap = static_cast<int>(glk::COLORMAP::TURBO);
  Eigen::Vector4f flat_color(1.0f, 0.5f, 0.2f, 1.0f);
  float point_size = 0.05f;
  bool metric = true;
  bool circle = true;
  bool y_up = true;  // The bunny model is Y-up
  bool show_normals = false;

  // Register the point cloud to the viewer with the current settings
  const auto update_points = [&] {
    // Model matrix that fits the point cloud into a 10m box around the origin
    Eigen::Affine3f model = Eigen::UniformScaling<float>(10.0f / cloud->extent) * Eigen::Translation3f(-cloud->center);
    if (y_up) {
      model = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * model;
    }

    // glk::PointCloudBuffer holds the points and their attributes on the GPU memory
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(cloud->vertices);
    if (!cloud->normals.empty()) {
      cloud_buffer->add_normals(cloud->normals);  // Normals are used by screen space effects (e.g., lighting)
    }

    guik::ShaderSetting setting;
    switch (color_mode) {
      case 0:  // Color encodes the height (Z) of each point
        setting = guik::Rainbow(model).set_rainbow_range(Eigen::Vector2f(-5.0f, 5.0f));
        break;
      case 1:  // Single color
        setting = guik::FlatColor(flat_color, model);
        break;
      case 2:  // Per-point RGBA colors
        cloud_buffer->add_color(cloud->colors);
        setting = guik::VertexColor(model);
        break;
      case 3:  // Per-point intensities converted to RGBA colors with a colormap on CPU
        cloud_buffer->add_intensity(static_cast<glk::COLORMAP>(colormap), cloud->intensities);
        setting = guik::VertexColor(model);
        break;
      case 4:  // Per-point scalar values converted to colors with the viewer's colormap on GPU
        cloud_buffer->add_colormap(cloud->cmaps);
        setting = guik::VertexColorMap(model).set_colormap_range(Eigen::Vector2f(0.0f, 0.5f * cloud->extent));
        break;
    }

    // Point shape: size in meters (metric) or pixels (screenspace), and circle or rectangle
    setting.set_point_shape(point_size, metric, circle);

    // Colormap used for RAINBOW and VERTEX_COLORMAP
    viewer->set_colormap(static_cast<glk::COLORMAP>(colormap));
    viewer->update_drawable("points", cloud_buffer, setting);

    if (show_normals && !cloud->normals.empty()) {
      viewer->update_drawable("normals", std::make_shared<glk::PointNormalsBuffer>(cloud->vertices, cloud->normals, 0.01 * cloud->extent), guik::FlatColor(0.2f, 1.0f, 0.2f, 1.0f, model));
    } else {
      viewer->remove_drawable("normals");
    }
  };
  update_points();

  viewer->register_ui_callback("points_ui", [&] {
    ImGui::Begin("points", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // File IO
    ImGui::Text("%s (%d points)", ply_path.c_str(), static_cast<int>(cloud->vertices.size()));
    bool changed = false;
    if (ImGui::Button("Load PLY")) {
      // The file dialog starts with the current file selected
      const auto results = pfd::open_file("Select a PLY file", ply_path, {"PLY files", "*.ply", "All files", "*"}).result();
      if (!results.empty()) {
        if (auto loaded = load_points(results[0])) {
          cloud = loaded;
          ply_path = results[0];
          changed = true;
        } else {
          viewer->append_text("failed to load " + results[0]);
        }
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save PLY")) {
      const auto path = pfd::save_file("Select a destination path", "points.ply", {"PLY files", "*.ply"}).result();
      if (!path.empty()) {
        // Save the points with the attributes
        glk::PLYData ply;
        ply.vertices = cloud->vertices;
        ply.normals = cloud->normals;
        ply.colors = cloud->colors;
        ply.intensities = cloud->intensities;
        ply.add_prop<float>("cmap", cloud->cmaps.data(), cloud->cmaps.size());  // Generic property
        viewer->append_text(glk::save_ply(path, ply) ? "saved to " + path : "failed to save " + path);
      }
    }

    // Rendering settings
    ImGui::Separator();
    changed |= ImGui::Combo("color mode", &color_mode, color_modes.data(), color_modes.size());
    changed |= ImGui::Combo("colormap", &colormap, colormap_names.data(), colormap_names.size());
    changed |= ImGui::ColorEdit4("flat color", flat_color.data());

    ImGui::Separator();
    changed |= ImGui::DragFloat("point size", &point_size, metric ? 0.001f : 0.1f, 0.001f, 100.0f);
    if (ImGui::Checkbox("metric", &metric)) {
      point_size = metric ? 0.05f : 5.0f;  // meters or pixels
      changed = true;
    }
    ImGui::SameLine();
    changed |= ImGui::Checkbox("circle", &circle);

    ImGui::Separator();
    changed |= ImGui::Checkbox("Y-up", &y_up);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("show normals", &show_normals);
    ImGui::End();

    if (changed) {
      update_points();
    }
  });

  viewer->spin();

  return 0;
}
