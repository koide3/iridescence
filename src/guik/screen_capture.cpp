#include <guik/screen_capture.hpp>

#include <glk/io/png_io.hpp>
#include <glk/io/jpeg_io.hpp>
#include <glk/pixel_buffer.hpp>
#include <glk/console_colors.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace guik {

ScreenCapture::ScreenCapture(
  const Eigen::Vector2i& image_size,
  int num_encode_threads,
  int num_pixel_buffers,
  std::shared_ptr<guik::ConcurrentQueue<std::pair<std::string, PixelsPtr>>> queue)
: image_size(image_size),
  capture_count(0),
  kill_switch(false),
  input_queue(queue) {
  dst_filenames.resize(num_encode_threads);
  pixel_buffers.resize(num_pixel_buffers);

  for (auto& pixel_buffer : pixel_buffers) {
    pixel_buffer = std::make_shared<glk::PixelBuffer>(image_size, sizeof(unsigned char) * 4);
  }

  encode_threads.resize(num_encode_threads);
  for (auto& thread : encode_threads) {
    thread = std::thread([this] { encode_task(); });
  }
}

ScreenCapture::~ScreenCapture() {
  for (int i = 0; i < pixel_buffers.size(); i++) {
    if (dst_filenames[i].empty()) {
      continue;
    }

    auto pixels = std::make_shared<std::vector<unsigned char>>(std::move(pixel_buffers[i]->read_pixels<unsigned char>()));
    input_queue->push(std::make_pair(dst_filenames[i], pixels));
  }

  kill_switch = true;
  for (auto& thread : encode_threads) {
    thread.join();
  }
}

size_t ScreenCapture::queue_size() const {
  return input_queue->size();
}

void ScreenCapture::capture(const std::string& dst_filename) {
  auto viewer = guik::viewer();
  capture(dst_filename, viewer->color_buffer());
}

void ScreenCapture::capture(const std::string& dst_filename, const glk::Texture& texture) {
  const int current = capture_count % pixel_buffers.size();
  if (texture.size() != image_size) {
    std::cerr << glk::console::yellow << "warning image_size mismatch!!" << glk::console::reset << std::endl;
    return;
  }

  if (!dst_filenames[current].empty()) {
    const auto& dst_filename = dst_filenames[current];
    auto pixels = std::make_shared<std::vector<unsigned char>>(std::move(pixel_buffers[current]->read_pixels<unsigned char>()));
    input_queue->push(std::make_pair(dst_filename, pixels));
  }

  dst_filenames[current] = dst_filename;
  pixel_buffers[current]->copy_from_texture(texture, GL_RGBA, GL_UNSIGNED_BYTE);

  capture_count++;
}

void ScreenCapture::encode_task() {
  while (true) {
    auto inputs = input_queue->try_pop(4);

    if (inputs.empty()) {
      if (kill_switch) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    for (const auto& [dst_filename, pixels] : inputs) {
      std::vector<unsigned char> row_buffer(4 * image_size[0]);
      for (int y_a = 0; y_a < image_size[1] / 2; y_a++) {
        const int y_b = image_size[1] - y_a - 1;

        auto row_a = pixels->data() + sizeof(unsigned char) * 4 * image_size[0] * y_a;
        auto row_b = pixels->data() + sizeof(unsigned char) * 4 * image_size[0] * y_b;

        memcpy(row_buffer.data(), row_a, sizeof(unsigned char) * 4 * image_size[0]);
        memcpy(row_a, row_b, sizeof(unsigned char) * 4 * image_size[0]);
        memcpy(row_b, row_buffer.data(), sizeof(unsigned char) * 4 * image_size[0]);
      }

      if (dst_filename.substr(dst_filename.size() - 4) == ".png") {
        if (!glk::save_png(dst_filename, image_size[0], image_size[1], *pixels)) {
          std::cerr << glk::console::yellow << "warning: failed to save " << dst_filename << glk::console::reset << std::endl;
        }
      } else if (dst_filename.substr(dst_filename.size() - 4) == ".jpg") {
        if (!glk::save_jpeg(dst_filename, image_size[0], image_size[1], *pixels)) {
          std::cerr << glk::console::yellow << "warning: failed to save " << dst_filename << glk::console::reset << std::endl;
        }
      } else {
        std::cerr << glk::console::yellow << "warning: unknown image file extension " << dst_filename << glk::console::reset << std::endl;
      }
    }
  }
}
}