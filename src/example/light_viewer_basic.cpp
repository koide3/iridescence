#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  float scale = 1.0f;
  viewer->register_ui_callback("ui", [&]() {
    ImGui::DragFloat("Scale", &scale, 0.01f, 0.01f, 10.0f);

    if(ImGui::Button("Close")) {
      viewer->close();
    }
  });

  while(viewer->spin_once()) {
    Eigen::Matrix4f transform = (Eigen::AngleAxisf(ImGui::GetTime(), Eigen::Vector3f::UnitZ()) * Eigen::UniformScaling<float>(scale) * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::Rainbow(transform));
  }

  return 0;
}