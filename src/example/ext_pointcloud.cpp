#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  const std::string path = "/home/koide/data2.ply";

  auto viewer = guik::LightViewer::instance();
  viewer->enable_decimal_rendering();

  auto ply = glk::load_ply(path);

  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  cloud_buffer->enable_decimal_rendering(8192 * 10);

  viewer->update_drawable("cloud", cloud_buffer, guik::Rainbow());
  viewer->spin();

  return 0;
}