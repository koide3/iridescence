#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  const std::string path = "/home/koide/datasets/tagl/bunny.ply";
  std::cout << path << std::endl;

  auto viewer = guik::LightViewer::instance();

  auto ply = glk::load_ply(path);

  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, ply->intensities);

  viewer->update_drawable("cloud", cloud_buffer, guik::VertexColor());
  viewer->spin();

  return 0;
}