# Multi-threading

## Invoke

Because OpenGL commands can only be executed on a single GUI context thread, most of functionalities of Iridescence are also required to be executed on the single GUI thread. For safely updating visualization contents, Iridescence provides ```invoke()``` method that pushes a task (functor) to a thread-safe task queue and requests the viewer to execute it on the GUI thread.

```cpp

// This function is executed in a background thread
void async_update() {
  for (int i=0; i<100; i++) {
    // Prepare point cloud data to be rendered
    std::shared_ptr<std::vector<Eigen::Vector3f>> points = ...;

    // Creating a PointCloudBuffer here is not allowed because OpenGL objects
    // must be created in the thread where the OpenGL context was created.
    // 
    // The following line does not work here!!
    // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(*points);

    auto viewer = guik::LightViewer::instance();
    // Requst the viewer to execute the following function in the GUI thread.
    viewer->invoke([=] {
      // This lambda function will be evaluated in the GUI thread lately.
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(*points);
      viewer->update_drawable("poits", cloud_buffer, guik::Rainbow());
    });
  }
}

int main(int argc, char** argv) {
  // OpenGL context is created on the thread where the first call of LightViewer::instance() was made.
  // We thus recommend calling it in the main thread before using any visualization operations.
  auto viewer = guik::LightViewer::instance();

  std::thread thread(async_update);

  viewer->spin();

  return 0;
}
```

## Thread-safe operations

```append_text``` and ```clear_text``` are thread-safe.

```cpp
viewer->append_text("test");
viewer->clear_text();
```

## AsyncViewer

If you want to keep the viewer interactive while your program is doing some blocking operations, `guik::AsyncLightViewer` would be helpful. This class creates the viewer instance in a background thread and performs visualization tasks in a way thread safe. See `06_light_viewer_async.cpp`.

```cpp
#include <guik/viewer/async_light_viewer.hpp>

int main(int argc, char** argv) {
  // AsyncViewer creates and runs the viewer in a background thread.
  auto async_viewer = guik::async_viewer();

  // Use AsyncViewer interfaces for safe viewer data manipulation.
  // AsyncViewer will be kept interactive even while the main thread is sleeping.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere1", guik::FlatRed().translate(0.0f, 0.0f, 0.0f));

  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere2", guik::FlatGreen().translate(2.0f, 0.0f, 0.0f));

  // Wait for the viewer to be closed.
  guik::async_wait();
}
```

!!! warning
    Because AsyncViewer runs the viewer in a background thread, calling the standard viewer functions in this main thread is unsafe.

    ```cpp
      auto async_viewer = guik::async_viewer();

      // The below line may cause segfaults and program crashes.
      // guik::viewer()->update_sphere("sphere0", guik::FlatBlue());
    ```
