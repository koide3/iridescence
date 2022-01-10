#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->show_info_window();

  float time = 0.0f;
  viewer->register_ui_callback("ui", [&]() {
    ImGui::DragFloat("Time", &time, 0.01f);

    if (ImGui::Button("Close")) {
      viewer->close();
    }
  });

  while (viewer->spin_once()) {
    Eigen::AngleAxisf transform(time, Eigen::Vector3f::UnitZ());
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow(transform));
  }

  return 0;
}