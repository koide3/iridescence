#include <random>
#include <chrono>
#include <thread>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

void run() {
  std::mt19937 mt;

  for(int i = 0; i < 500; i++) {
    std::vector<float> points(128 * 3);
    for(int i = 0; i < points.size(); i++) {
      points[i] = std::uniform_real_distribution<>(-10.0, 10.0)(mt);
    }

    // the following line doesn't work because GL objects must be creaded in the GUI thread
    // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points.data(), sizeof(float) * 3, points.size() / 3);

    // request to invoke a task in the GUI thread
    guik::LightViewer::instance()->invoke([=] {
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points.data(), sizeof(float) * 3, points.size() / 3);
      guik::LightViewer::instance()->update_drawable("cloud_" + std::to_string(i), cloud_buffer, guik::FlatColor(Eigen::Vector4f::Random() * 2.0f).add("point_scale", 3.0f));
    });

    // ```append_text``` and ```clear_texts``` are thread-safe
    guik::LightViewer::instance()->append_text(std::to_string(i) + " : add points");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char** argv) {
  // the fist call of ```guik::LightViewer::instance()``` in the main thread
  auto viewer = guik::LightViewer::instance();

  std::thread thread(run);

  viewer->spin();
  return 0;
}