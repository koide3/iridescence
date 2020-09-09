#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  viewer->register_ui_callback("ui", [&]() {
    if(ImGui::Button("Close")) {
      viewer->close();
    }
  });

  while(viewer->spin_once()) {
    Eigen::Matrix4f transform = (Eigen::AngleAxisf(ImGui::GetTime(), Eigen::Vector3f::UnitZ()) * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::Rainbow(transform));

    auto sub_viewer = viewer->sub_viewer("sub", Eigen::Vector2i(512, 512));
    sub_viewer->update_drawable("sphere", glk::Primitives::primitive_ptr(glk::Primitives::SPHERE), guik::Rainbow());
  }

  return 0;
}