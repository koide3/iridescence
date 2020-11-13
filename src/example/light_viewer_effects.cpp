#include <glk/primitives/primitives.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>
#include <glk/effects/screen_space_lighting.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->show_info_window();

  int effect = 0;
  viewer->register_ui_callback("effect_ui", [&]() {
    std::vector<const char*> effects = {"PLAIN", "SSAO", "SS_LIGHTING"};
    if(ImGui::Combo("Effect", &effect, effects.data(), effects.size())) {
      switch(effect) {
        case 0:
          viewer->set_screen_effect(std::make_shared<glk::PlainRendering>());
          break;
        case 1:
          viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>());
          break;
        case 2:
          viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size()));
          break;
      }
    }
  });

  viewer->update_drawable("floor", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::Rainbow((Eigen::Scaling<float>(25.0f, 25.0f, 0.1f) * Eigen::Isometry3f::Identity()).matrix()));

  Eigen::Matrix4f transform = (Eigen::Translation3f(0.0f, 0.0f, -0.5f) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(15.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("bunny", glk::Primitives::primitive_ptr(glk::Primitives::BUNNY), guik::Rainbow(transform));

  viewer->spin();
  return 0;
}