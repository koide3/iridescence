#include <iostream>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->show_info_window();
  viewer->spin();
  return 0;
}