#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  // Create a viewer instance (global singleton)
  auto viewer = guik::LightViewer::instance();

  float angle = 0.0f;

  // Register a callback for UI rendering
  viewer->register_ui_callback("ui", [&]() {
    // In the callback, you can call ImGui commands to create your UI.
    // Here, we use "DragFloat" and "Button" to create a simple UI.
    ImGui::DragFloat("Angle", &angle, 0.01f);

    if (ImGui::Button("Close")) {
      viewer->close();
    }
  });

  // Spin the viewer until it gets closed
  while (viewer->spin_once()) {
    // Objects to be rendered are called "drawables" and managed with unique names.
    // Here, solid and wire spheres are registered to the viewer respectively with the "Rainbow" and "FlatColor" coloring schemes.
    // The "Rainbow" coloring scheme encodes the height of each fragment using the turbo colormap by default.
    Eigen::AngleAxisf transform(angle, Eigen::Vector3f::UnitZ());
    viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow(transform));
    viewer->update_drawable("wire_sphere", glk::Primitives::wire_sphere(), guik::FlatColor({0.1f, 0.7f, 1.0f, 1.0f}, transform));
  }

  return 0;
}