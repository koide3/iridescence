# Point picking

## Getting the clicked position

```cpp
viewer->register_ui_callback("ui", [&] {
  auto& io = ImGui::GetIO();
  // If right clicked the GL canvas
  if (!io.WantCaptureMouse && io.MouseClicked[ImGuiMouseButton_Right]) {
    // Pick the depth of the clicked pixel
    float depth = viewer->pick_depth({io.MousePos.x, io.MousePos.y});

    // If depth < 1.0f, the clicked pixel is a foreground object. Otherwise, it is the background.
    if(depth < 1.0f) {
      // Compute the 3D position of the clicked pixel
      Eigen::Vector3f pos = viewer->unproject({io.MousePos.x, io.MousePos.y}, depth);
      viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::FlatRed().translate(pos).scale(0.05));
    }
  }
});
```

![picking](https://user-images.githubusercontent.com/31344317/210159144-bacd830b-aad9-4167-9a2d-c233efc4a0ff.gif)

There is also a convenient method ```pick_point()``` that check if the clicked point has valid 3D coordinates.
```cpp
viewer->register_ui_callback("ui", [&] {
  int button = ImGuiMouseButton_Right;
  std::optional<Eigen::Vector3f> pt = viewer->pick_point(button);
  if (pt) {
    std::cout << "clicked point=" << pt->transpose() << std::endl;
  }
}
```


## Getting the clicked object information

```cpp
// Enable information buffer
viewer->enable_info_buffer();

// Draw green and blue cubes with info_values == {1, 0, 0, 0} and {2, 0, 0, 0} respectively
viewer->update_drawable("cube1", glk::Primitives::cube(),
  guik::FlatGreen().add("info_values", Eigen::Vector4i(1, 0, 0, 0)));
viewer->update_drawable("cube2", glk::Primitives::cube(),
  guik::FlatBlue().add("info_values", Eigen::Vector4i(2, 0, 0, 0)).translate({2.0, 0.0, 0.0}));

viewer->register_ui_callback("ui", [&] {
  auto& io = ImGui::GetIO();
  if (!io.WantCaptureMouse && io.MouseClicked[ImGuiMouseButton_Right]) {
    float depth = viewer->pick_depth({io.MousePos.x, io.MousePos.y});

    // If the clicked pixel is a foreground object
    if (depth < 1.0f) {
      // Pick the info_values of the clicked pixel
      Eigen::Vector4i info = viewer->pick_info({io.MousePos.x, io.MousePos.y});

      if (info[0] == 1) {
        viewer->append_text("Green cube clicked!!");
      } else if (info[0] == 2) {
        viewer->append_text("Blue cube clicked!!");
      }
    }
  }
});

```

![info_picking](https://user-images.githubusercontent.com/31344317/210159319-9896be23-adff-4a07-a790-6017b64fa06b.gif)
