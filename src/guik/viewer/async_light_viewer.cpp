#include <guik/viewer/async_light_viewer.hpp>

#include <atomic>

namespace guik {

std::unique_ptr<AsyncLightViewer> AsyncLightViewer::inst;

AsyncLightViewer::AsyncLightViewer(const Eigen::Vector2i& size, bool background, const std::string& title) {
  std::atomic_bool initiated = false;

  kill_switch = false;
  thread = std::thread([this, size, background, title, &initiated] {
    auto viewer = guik::viewer(size, background, title);
    context = viewer;

    initiated = true;

    while (!kill_switch && viewer->spin_once()) {
    }
    guik::destroy();
  });

  while (!initiated) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

AsyncLightViewer::~AsyncLightViewer() {
  kill_switch = true;
  thread.join();
}

AsyncLightViewer* AsyncLightViewer::instance(const Eigen::Vector2i& size, bool background, const std::string& title) {
  if (!inst) {
    inst.reset(new AsyncLightViewer(size, background, title));
  }
  return inst.get();
}

void AsyncLightViewer::destroy() {
  inst.reset();
}

void AsyncLightViewer::wait() {
  inst->thread.join();
  inst.reset();
}

void AsyncLightViewer::invoke(const std::function<void()>& func) {
  guik::viewer()->invoke(func);
}

void AsyncLightViewer::invoke_after_rendering(const std::function<void()>& func) {
  guik::viewer()->invoke_after_rendering(func);
}

AsyncLightViewerContext AsyncLightViewer::async_sub_viewer(const std::string& context_name, const Eigen::Vector2i& canvas_size) {
  std::shared_ptr<guik::LightViewerContext> sub_viewer;
  guik::viewer()->invoke([&] { sub_viewer = guik::viewer()->sub_viewer(context_name, canvas_size); });
  while (!sub_viewer) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  }

  return AsyncLightViewerContext(sub_viewer.get());
}

}  // namespace guik
