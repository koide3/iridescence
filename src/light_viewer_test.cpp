#include <iostream>

#include <glk/primitives/primitives.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->spin();
  return 0;
}