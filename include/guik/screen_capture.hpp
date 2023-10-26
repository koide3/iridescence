#ifndef GUIK_SCREEN_CAPTURE
#define GUIK_SCREEN_CAPTURE

#include <deque>
#include <memory>
#include <atomic>
#include <thread>
#include <iostream>
#include <Eigen/Core>
#include <guik/concurrent_queue.hpp>

namespace glk {
class Texture;
class PixelBuffer;
}  // namespace glk

namespace guik {

/**
 * @brief Screen capture utility to asynchronously fetch rendering results and save them as images.
 * @note  The saving process is very computationally heavy as it encodes each frame as PNG or JPG.
*/
class ScreenCapture {
public:
  using Pixels = std::vector<unsigned char>;
  using PixelsPtr = std::shared_ptr<Pixels>;
  using ImageSaveData = std::pair<std::string, PixelsPtr>;

  /**
   * @brief Constructor
   * @param image_size          Image size
   * @param num_encode_threads  Number of threds for image encoding
   * @param num_pixel_buffers   Number of pixel buffers
   * @param queue               Concurrent data queue (Use TbbConcurrentQueue for better multi-threading performance)
  */
  ScreenCapture(
    const Eigen::Vector2i& image_size,
    int num_encode_threads,
    int num_pixel_buffers = 2,
    std::shared_ptr<guik::ConcurrentQueue<ImageSaveData>> queue = std::make_shared<guik::StdConcurrentQueue<ImageSaveData>>());
  ~ScreenCapture();

  /**
   * @brief Image encoding queue size
  */
  size_t queue_size() const;

  /**
   * @brief Capture the viewer main canvas and save it as a image file.
   * @param dst_filename  Filename (must end with ".png" or ".jpg")
  */
  void capture(const std::string& dst_filename);

  /**
   * @brief Capture texture contents and save it as a image file.
   * @param dst_filename  Filename (must end with ".png" or ".jpg")
   */
  void capture(const std::string& dst_filename, const glk::Texture& texture);

private:
  void encode_task();

private:
  const Eigen::Vector2i image_size;

  size_t capture_count;
  std::vector<std::string> dst_filenames;
  std::vector<std::shared_ptr<glk::PixelBuffer>> pixel_buffers;

  std::atomic_bool kill_switch;
  std::vector<std::thread> encode_threads;

  std::shared_ptr<guik::ConcurrentQueue<ImageSaveData>> input_queue;
};

}  // namespace guik

#endif