#include <chrono>
#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/io/png_io.hpp>
#include <glk/make_shared.hpp>
#include <glk/texture.hpp>
#include <glk/splatting.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->set_draw_xy_grid(false);
  viewer->enable_normal_buffer();

  auto ply = glk::load_ply("/home/koide/datasets/spla/e232_normal.ply");
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  cloud_buffer->add_normals(ply->normals);
  cloud_buffer->add_color(ply->colors);

  std::vector<float> radius(ply->vertices.size());
  for (int i = 0; i < radius.size(); i++) {
    radius[i] = static_cast<double>(i) / radius.size();
  }
  cloud_buffer->add_buffer("vert_radius", 1, radius.data(), sizeof(float), radius.size());

  int width, height;
  std::vector<unsigned char> bytes;
  glk::load_png("/home/koide/splat_00.png", width, height, bytes);
  auto texture = std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, bytes.data());

  auto splats = std::make_shared<glk::Splatting>();
  // splats->enable_vertex_radius();
  splats->set_cloud_buffer(cloud_buffer);
  splats->set_point_radius(0.1);
  // splats->set_texture(texture);

  // viewer->update_drawable("points", cloud_buffer, guik::Rainbow());

  // viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
  viewer->update_drawable("splats", splats, guik::VertexColor());
  viewer->spin();
  return 0;
}