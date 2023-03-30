#include <glk/primitives/primitives.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->show_info_window();

  int effect = 0;
  viewer->register_ui_callback("effect_ui", [&]() {
    std::vector<const char*> effects = {"PLAIN", "NAIVE_SSAO", "NORMAL", "SSAO", "SSLI"};
    if (ImGui::Combo("Effect", &effect, effects.data(), effects.size())) {
      switch (effect) {
        case 0:
          viewer->set_screen_effect(std::make_shared<glk::PlainRendering>());
          break;
        case 1:
          viewer->set_screen_effect(std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>());
          break;
        case 2:
          viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAttributeEstimation>(viewer->canvas_size(), glk::ScreenSpaceAttributeEstimation::BufferType::NORMAL));
          break;
        case 3:
          viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>(viewer->canvas_size()));
          break;
        case 4:
          auto effect = std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size());
          effect->set_light(0, Eigen::Vector3f(-1.0f, -1.0f, 10.0f), Eigen::Vector4f::Constant(1.6f));
          viewer->set_screen_effect(effect);
          break;
      }
    }
  });

  viewer->update_drawable("floor", glk::Primitives::cube(), guik::Rainbow().scale(25.0f, 25.0f, 0.1f));
  viewer->update_drawable("bunny", glk::Primitives::bunny(), guik::Rainbow(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(15.0f)));

  viewer->spin();
  return 0;
}