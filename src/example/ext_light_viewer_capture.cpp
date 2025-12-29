#include <glk/io/png_io.hpp>
#include <glk/pixel_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {

  auto viewer = guik::LightViewer::instance();
  viewer->disable_vsync();
  viewer->show_info_window();
  viewer->update_drawable("bunny", glk::Primitives::wire_bunny(), guik::Rainbow(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(50.0f)));

  const Eigen::Vector2i image_size = viewer->framebuffer_size();
  std::vector<unsigned char> color_pixels;
  std::vector<float> depth_pixels;

  std::vector<const char*> capture_modes = {"DIRECT_READ", "PIXEL_BUFFER"};
  int capture_mode = 0;
  bool capture_depth = false;

  viewer->register_ui_callback("ui", [&] {
    ImGui::Combo("capture_mode", &capture_mode, capture_modes.data(), capture_modes.size());
    ImGui::Checkbox("capture_depth", &capture_depth);

    if(ImGui::Button("save")) {
      if(!color_pixels.empty()) {
        glk::save_png("/tmp/color.png", image_size[0], image_size[1], color_pixels);
        viewer->append_text("saved to /tmp/color.png");
      }
    }
  });

  // Create pixel buffers
  int count = 0;
  const int num_pixel_buffers = 2;
  std::vector<std::shared_ptr<glk::PixelBuffer>> pixel_buffers(num_pixel_buffers);
  std::vector<std::shared_ptr<glk::PixelBuffer>> depth_pixel_buffers(num_pixel_buffers);
  for(int i=0; i<num_pixel_buffers; i++) {
    pixel_buffers[i] = glk::make_shared<glk::PixelBuffer>(image_size, sizeof(unsigned char) * 4);
    depth_pixel_buffers[i] = glk::make_shared<glk::PixelBuffer>(image_size, sizeof(float));
  }

  while(viewer->spin_once()) {
    if(capture_mode == 0) {
      // Read pixels using glGetTexImage
      color_pixels = viewer->read_color_buffer();
      if(capture_depth) {
        depth_pixels = viewer->read_depth_buffer();
      }
    } else {
      // Read pixels via asynchronous data transfer using PBO
      // Note: glk::PixelBuffer provides only a low-level interface, and captured images are flipped vertically
      int current = count % num_pixel_buffers;
      pixel_buffers[current]->copy_from_texture(viewer->color_buffer(), GL_RGBA, GL_UNSIGNED_BYTE);
      if(count > 0) {
        int prev = (count - 1) % num_pixel_buffers;
        color_pixels = pixel_buffers[prev]->read_pixels<unsigned char>();
      }

      if(capture_depth) {
        depth_pixel_buffers[current]->copy_from_texture(viewer->depth_buffer(), GL_DEPTH_COMPONENT, GL_FLOAT);
        if(count > 0) {
          int prev = (count - 1) % num_pixel_buffers;
          depth_pixels = depth_pixel_buffers[prev]->read_pixels<float>();
        }
      }
      count++;
    }

  }

  return 0;
}