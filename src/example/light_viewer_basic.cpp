#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/recent_files.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  viewer->register_ui_callback("ui", [&]() {
    ImGui::Text("Basic example");
    if(ImGui::Button("Close")) {
      viewer->close();
    }
  });

  while(viewer->spin_once()) {
    Eigen::Matrix4f transform = (Eigen::AngleAxisf(ImGui::GetTime(), Eigen::Vector3f::UnitZ()) * Eigen::Isometry3f::Identity()).matrix();
    viewer->update_drawable("icosahedron", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::Rainbow(transform));
  }

  return 0;
}