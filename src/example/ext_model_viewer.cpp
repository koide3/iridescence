#include <iostream>
#include <glk/io/mesh_io.hpp>
#include <glk/io/image_io.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->enable_normal_buffer();
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
  });

  viewer->spin();

  return 0;
}