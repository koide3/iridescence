#include <glk/primitives/primitives.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  bool enable_ssao = false;
  viewer->register_ui_callback("effect_ui", [&]() {
    if(ImGui::Checkbox("SSAO", &enable_ssao)) {
      if(enable_ssao) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>());
      } else {
        viewer->set_screen_effect(std::make_shared<glk::PlainRendering>());
      }
    }
  });

  for(int i = 0; i < 10; i++) {
    viewer->update_drawable("icosahedron_" + std::to_string(i), glk::Primitives::primitive_ptr(glk::Primitives::ICOSAHEDRON), guik::Rainbow((Eigen::Translation3f(0, i * 0.1, i) * Eigen::Isometry3f::Identity()).matrix()));
  }
  viewer->spin();
  return 0;
}