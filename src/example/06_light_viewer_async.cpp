#include <thread>
#include <glk/thin_lines.hpp>
#include <glk/delayed_constructor.hpp>
#include <guik/viewer/async_light_viewer.hpp>

int main(int argc, char** argv) {
  // AsyncViewer creates and runs the viewer in a background thread.
  // This is helpful in situations you want to keep the viewer interactive while the main thread is doing some blocking operations.
  // Note that this "fire-and-forget" style usage brings some limitations of possible operations and data copy overhead.
  auto async_viewer = guik::async_viewer();

  // Because the viewer is running in another thread, calling the standard viewer functions in this main thread is unsafe.
  // Thus, the below line may cause segfaults and program crashes.
  // guik::viewer()->update_sphere("sphere0", guik::FlatBlue());

  // Use AsyncViewer interfaces for safe viewer data manipulation.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere1", guik::FlatRed().translate(0.0f, 0.0f, 0.0f));

  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere2", guik::FlatGreen().translate(2.0f, 0.0f, 0.0f));

  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere3", guik::FlatBlue().translate(4.0f, 0.0f, 0.0f));

  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::vector<Eigen::Vector3f> line;
  for (double y = -1.0; y <= 1.0; y += 0.01) {
    line.emplace_back(std::sin(y * 10.0) * 0.1 + 6.0, y, 0.0);
  }

  // We cannot directly create glk::ThinLines here because it creates OpenGL buffers that must be done in the viewer thread
  // auto dline = std::make_shared<glk::ThinLines>(line, true);

  // Instead, we can use glk::DelayedConstructor to make the construction in the viewer thread in lazy manner
  auto dline = std::make_shared<glk::DelayedConstructor>([line] { return std::make_shared<glk::ThinLines>(line, true); });
  async_viewer->update_drawable("line", dline, guik::FlatOrange());

  // Register a UI callback, which will be called in the viewer thread.
  async_viewer->register_ui_callback("ui_callback", [] {
    if (ImGui::Button("Remove red sphere")) {
      // Because this callback will be called in the viewer thread,
      // it is safe to use the standard viewer interfaces.
      guik::viewer()->remove_drawable("sphere1");
    }
  });

  // ```async_sub_viewer``` causes a syncronization with the visualization thread.
  // Do not call it frequently.
  auto async_sub_viewer = async_viewer->async_sub_viewer("sub_viewer");
  async_sub_viewer.update_wire_icosahedron("icosahedron", guik::FlatRed());

  // Wait for the viewer to be closed.
  // To terminate the async viewer immediately, call ```async_destroy``` instead.
  guik::async_wait();

  return 0;
}