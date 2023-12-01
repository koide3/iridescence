#ifndef GUIK_ASYNC_LIGHT_VIEWER_HPP
#define GUIK_ASYNC_LIGHT_VIEWER_HPP

#include <atomic>
#include <thread>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer_context.hpp>

namespace guik {

class AsyncLightViewer : public AsyncLightViewerContext {
private:
  AsyncLightViewer(const Eigen::Vector2i& size, bool background, const std::string& title);

public:
  virtual ~AsyncLightViewer();

  /// @brief Create and run the viewer in a background thread
  static AsyncLightViewer* instance(const Eigen::Vector2i& size = Eigen::Vector2i(-1, -1), bool background = false, const std::string& title = "screen");

  /// @brief Destroy the viewer in the background thread
  static void destroy();

  /// @brief Wait for the viewer to be closed
  static void wait();

  void invoke(const std::function<void()>& func);
  void invoke_after_rendering(const std::function<void()>& func);

  // This method causes a synchronization with the visualization thread.
  // Do not call this frequently.
  AsyncLightViewerContext async_sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size = Eigen::Vector2i(-1, -1));

private:
  static std::unique_ptr<AsyncLightViewer> inst;

  std::atomic_bool kill_switch;
  std::thread thread;
};

inline AsyncLightViewer* async_viewer(const Eigen::Vector2i& size = Eigen::Vector2i(-1,  -1), bool background = false, const std::string& title = "screen") {
  return AsyncLightViewer::instance(size, background, title);
}

inline void async_destroy() {
  AsyncLightViewer::destroy();
}

inline void async_wait() {
  AsyncLightViewer::wait();
}

}  // namespace guik

#endif