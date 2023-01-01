#include <glk/primitives/primitives.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->set_draw_xy_grid(false);

  auto trans = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(10.0f);
  viewer->update_drawable("cube", glk::Primitives::bunny(), guik::Rainbow(trans));

  viewer->register_ui_callback("ui", [=] {
    if(ImGui::Button("Enable normal buffer")) {
      viewer->enable_normal_buffer();
    }

    if (ImGui::Button("Naive SSAO")) {
      auto ssao = std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>();
      viewer->set_screen_effect(ssao);
    }

    if (ImGui::Button("SSAO")) {
      auto ssao = std::make_shared<glk::ScreenSpaceAmbientOcclusion>();
      viewer->set_screen_effect(ssao);
    }

    if (ImGui::Button("Depth")) {
      auto ssae = std::make_shared<glk::ScreenSpaceAttributeEstimation>();
      ssae->set_rendering_buffer(glk::ScreenSpaceAttributeEstimation::BufferType::DEPTH);
      viewer->set_screen_effect(ssae);
    }

    if (ImGui::Button("Position")) {
      auto ssae = std::make_shared<glk::ScreenSpaceAttributeEstimation>();
      ssae->set_rendering_buffer(glk::ScreenSpaceAttributeEstimation::BufferType::POSITION);
      viewer->set_screen_effect(ssae);
    }

    if (ImGui::Button("Normal")) {
      auto ssae = std::make_shared<glk::ScreenSpaceAttributeEstimation>();
      ssae->set_rendering_buffer(glk::ScreenSpaceAttributeEstimation::BufferType::NORMAL);
      viewer->set_screen_effect(ssae);
    }

    if (ImGui::Button("Occulusion")) {
      auto ssae = std::make_shared<glk::ScreenSpaceAttributeEstimation>();
      ssae->set_rendering_buffer(glk::ScreenSpaceAttributeEstimation::BufferType::SSAO);
      viewer->set_screen_effect(ssae);
    }

    if (ImGui::Button("SSL")) {
      Eigen::Vector3f light0_pos(1.0f, 1.0f, 5.0f);
      Eigen::Vector4f light0_color(2.0f, 2.0f, 2.0f, 1.0f);

      Eigen::Vector3f light1_pos(0.0f, -2.0f, 2.0f);
      Eigen::Vector4f light1_color(0.5f, 0.5f, 0.5f, 1.0f);

      auto lighting_effect = std::make_shared<glk::ScreenSpaceLighting>();
      lighting_effect->set_light(0, light0_pos, light0_color);
      lighting_effect->set_light(1, light1_pos, light1_color);

      viewer->set_screen_effect(lighting_effect);
    }
  });


  viewer->spin();

  return 0;
}