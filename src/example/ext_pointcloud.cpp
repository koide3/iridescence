#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/mesh.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  const std::string path = "/home/koide/bunny.ply";
  std::cout << path << std::endl;

  auto viewer = guik::LightViewer::instance();
  viewer->enable_normal_buffer();

  auto ply = glk::load_ply(path);

  auto mesh = std::make_shared<glk::Mesh>(ply->vertices, ply->normals, ply->indices);

  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, ply->intensities);

  viewer->update_drawable("cloud", mesh, guik::FlatGreen());
  viewer->spin();

  return 0;
}