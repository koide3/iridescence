#include <iostream>

#include <glk/gridmap.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  int width = 100, height = 100;
  double resolution = 0.1;

  std::vector<unsigned char> color_map_rgba(height * width * 4);
  unsigned char r, g, b, a;
  b = 0;
  a = 255;
  for (int i = 0; i < height; i++) {
    r = i * (255 / (double)height);
    for (int j = 0; j < width; j++) {
      g = j * (255 / (double)width);
      
      int idx = (i * width + j) * 4;
      color_map_rgba[idx + 0] = r;
      color_map_rgba[idx + 1] = g;
      color_map_rgba[idx + 2] = b;
      color_map_rgba[idx + 3] = a;
    }
  }

  auto grid_map_rgba = std::make_shared<glk::GridMap>(resolution, width, height, color_map_rgba.data(), 255, glk::GridMap::ColorMode::RGBA);
  viewer->update_drawable("gridmap_rgba", grid_map_rgba, guik::TextureColor());

  std::vector<unsigned char> color_map_turbo(height * width);
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      color_map_turbo[i * width + j] = i + j;
    }
  }
  
  auto grid_map = std::make_shared<glk::GridMap>(resolution, width, height, color_map_turbo.data(), 255, glk::GridMap::ColorMode::TURBO);
  viewer->update_drawable("gridmap_turbo", grid_map, guik::TextureColor());

  int color_map_type = 0;
  viewer->register_ui_callback("colormap_selector", [&]() {
    ImGui::Combo("ColorMap", &color_map_type, "RGBA\0TURBO\0\0");
  });

  viewer->register_drawable_filter("drawable_filter", [&](const std::string& drawable_name){
    if (color_map_type != 0 && drawable_name.find("gridmap_rgba") != std::string::npos) {
      return false;
    }
    if (color_map_type != 1 && drawable_name.find("gridmap_turbo") != std::string::npos) {
      return false;
    }

    return true;
  });

  viewer->spin();

  return 0;
}
