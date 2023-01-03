# Camera and model controls

## Camera control

```cpp
auto viewer = guik::LightViewer::instance();

// Use a camera control that moves along with the XY-plane
viewer->use_orbit_camera_control();

// Use a camera control that moves along with the XZ-plane
viewer->use_orbit_camera_control_xz();

// Use a top-down camera control facing to the XY-plane
viewer->use_topdown_camera_control();

// Use an arcball-like camera control that can make an arbitrary pose
viewer->use_arcball_camera_control();
```

A useful method to let the camera keep focusing on a moving object is ```lookat()``` that moves the camera such that the specified position comes to the center of the camera view.

```cpp
Eigen::Vector3f center_pos = ...;
viewer->lookat(center_pos);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/TarRKF_Xd2E?start=13" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
```lookat``` example (0:13 ~)

## Model matrix control (ImGuizmo)

```cpp
#include <guik/model_control.hpp>

Eigen::Matrix4f init_model_matrix = Eigen::Matrix4f::Identity();
guik::ModelControl model_control("model_control", init_model_matrix);

viewer->register_ui_callback("model_control_ui", [&]{
  model_control.draw_gizmo_ui();
  model_control.draw_gizmo();

  Eigen::Matrix4f model_matrix = model_control.model_matrix();
  viewer->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow(model_matrix));
});
```

Note: ImGuizmo cannot be shown twice or more in one rendering frame.

![guizmo](https://user-images.githubusercontent.com/31344317/210159001-58b69d32-70b2-4fd1-9885-d40af93514d4.gif)
