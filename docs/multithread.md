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

```cpp
viewer->append_text("test");
viewer->clear_text();
```
