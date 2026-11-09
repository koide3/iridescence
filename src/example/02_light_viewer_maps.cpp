#include <glk/gridmap.hpp>
#include <glk/voxelmap.hpp>
#include <glk/elevation_map.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->disable_xy_grid();

  auto hover = std::make_shared<guik::HoveredDrawings>();
  viewer->register_ui_callback("hovered", hover->create_callback());

  const double resolution = 0.1;
  const double width_m = 5.0;
  const double height_m = 5.0;
  const int width = static_cast<int>(width_m / resolution);
  const int height = static_cast<int>(height_m / resolution);

  std::vector<Eigen::Vector3f> surface_points(width * height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double x_m = x * resolution;
      const double y_m = y * resolution;
      surface_points[y * width + x] = Eigen::Vector3f(x_m, y_m, 0.25 * std::sin(x_m * 3.0) * std::cos(y_m * 3.0));
    }
  }

  std::vector<float> elevation(surface_points.size());
  std::transform(surface_points.begin(), surface_points.end(), elevation.begin(), [](const Eigen::Vector3f& p) { return p.z(); });

  // glk::GridMap
  auto gridmap = std::make_shared<glk::GridMap>(resolution, width, height, 4.0, elevation.data(), 1.0, glk::GridMap::ColorMode::TURBO);
  viewer->update_drawable("gridmap", gridmap, guik::TextureColor().translate(-5.0f, -5.0f, 0.0f));
  viewer->update_coord("gridmap_coord", guik::VertexColor().translate(-5.0f, -5.0f, 0.0f));
  hover->add_text_on("gridmap", "2D grid map");

  // glk::ElevationMap (from 3D surface points)
  auto elev_map_builder = glk::ElevationMapBuilder(resolution, Eigen::Vector2i(width, height), Eigen::Vector2f(0.0f, 0.0f));
  const auto cell_indices = elev_map_builder.calc_cell_indices(surface_points);
  elev_map_builder.set_elevation(surface_points, 2, cell_indices);
  elev_map_builder.set_cmap(surface_points, 2, cell_indices);

  auto elev_map = std::make_shared<glk::ElevationMap>(elev_map_builder, glk::MeshRenderingOptions());
  viewer->update_drawable("elevation_map", elev_map, guik::VertexColorMap().translate(0.0f, -5.0f - width_m / 2.0f, 0.0f));
  viewer->update_coord("elevation_map_coord", guik::VertexColor().translate(0.0f, -5.0f - width_m / 2.0f, 0.0f));
  hover->add_text_on("elevation_map", "3D elevation map");

  // glk::ElevationMap (from 2D elevation map)
  auto elev_map2_builder = glk::ElevationMapBuilder(resolution, Eigen::Vector2i(width, height), Eigen::Vector2f(0.0f, 0.0f));
  elev_map2_builder.elevation_map = Eigen::Map<Eigen::Array<float, -1, -1, Eigen::RowMajor>>(elevation.data(), height, width);
  elev_map2_builder.color_map = 4.0f * Eigen::Map<Eigen::Array<float, -1, -1, Eigen::RowMajor>>(elevation.data(), height, width);

  auto elev_map2 = std::make_shared<glk::ElevationMap>(elev_map2_builder, glk::MeshRenderingOptions().set_edge_color({0.0f, 0.0f, 1.0f, 0.2f}));
  viewer->update_drawable("elevation_map2", elev_map2, guik::VertexColorMap().translate(0.0f, 0.0f, 0.0f));
  viewer->update_coord("elevation_map2_coord", guik::VertexColor().translate(0.0f, 0.0f, 0.0f));
  hover->add_text_on("elevation_map2", "3D elevation map");

  // glk::VoxelMap
  std::vector<Eigen::Vector3i> voxel_coords(surface_points.size());
  std::transform(surface_points.begin(), surface_points.end(), voxel_coords.begin(), [resolution](const Eigen::Vector3f& p) {
    return (p / resolution).array().floor().cast<int>();
  });

  auto voxelmap = std::make_shared<glk::VoxelMap>(voxel_coords, resolution, glk::MeshRenderingOptions().set_edge_color({0.0f, 0.0f, 1.0f, 1.0f}));
  viewer->update_drawable("voxelmap", voxelmap, guik::Rainbow().translate(7.5f, -5.0f - width_m / 2, 0.0f));
  viewer->update_coord("voxelmap_coord", guik::VertexColor().translate(7.5f, -5.0f - width_m / 2, 0.0f));
  hover->add_text_on("voxelmap", "3D voxel map");

  viewer->spin();

  return 0;
}