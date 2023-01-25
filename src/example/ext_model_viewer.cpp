#include <iostream>
#include <glk/io/mesh_io.hpp>
#include <glk/io/image_io.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>
#include <glk/effects/screen_space_lighting.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->use_arcball_camera_control();
  // viewer->enable_normal_buffer();
  viewer->disable_xy_grid();

  std::shared_ptr<glk::MeshModel> model;

  viewer->register_ui_callback("ui", [&] {
    if (ImGui::Button("Load model")) {
      auto path = pfd::open_file("Select a 3D model file").result();
      if (!path.empty()) {
        model = glk::load_mesh_model(path[0]);
        viewer->update_drawable("model", model, guik::Rainbow().rotate(M_PI_2, {1.0f, 0.0f, 0.0f}));
      }
    }

    if (ImGui::Button("Load texture") && model) {
      const auto path = pfd::open_file("Select a texture file").result();
      if (!path.empty()) {
        int width, height;
        std::vector<unsigned char> pixels;
        if (glk::load_image(path[0], width, height, pixels)) {
          auto texture = std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
          model->override_material(guik::TextureColor(), texture);
        }
      }
    }

    ImGui::Separator();
    if (ImGui::Button("Enable normal buffer")) {
      viewer->enable_normal_buffer();
    }

    if (ImGui::Button("Naive SSAO")) {
      viewer->set_screen_effect(std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>());
    }
    if (ImGui::Button("Smoothed SSAO")) {
      viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>());
    }
    if (ImGui::Button("Normal")) {
      auto effect = std::make_shared<glk::ScreenSpaceAttributeEstimation>();
      effect->set_rendering_buffer(glk::ScreenSpaceAttributeEstimation::BufferType::NORMAL);
      viewer->set_screen_effect(effect);
    }
    if (ImGui::Button("SSLI")) {
      auto lighting_effect = std::make_shared<glk::ScreenSpaceLighting>();
      lighting_effect->set_directional_light(0, {1.0f, 1.0f, 10.0f}, {2.0f, 2.0f, 2.0f, 1.0f});
      lighting_effect->set_directional_light(1, {0.0f, 2.0f, 2.0f}, {1.0f, 1.0f, 1.0f, 1.0f});
      viewer->set_screen_effect(lighting_effect);
    }
  });

  viewer->spin();

  return 0;
}