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


```cpp
Eigen::Vector3f center_pos = ...;
viewer->lookat(center_pos);
```

## Model matrix control (Gizmo)

```cpp
#include <guik/model_control.hpp>

Eigen::Matrix4f init_model_matrix = Eigen::Matrix4f::Identity();
guik::ModelControl model_control("model_control", init_model_matrix);

viewer->register_ui_callback("model_control_ui", [&]{
  model_control.draw_gizmo_ui();
  model_control.draw_gizmo();

  Eigen::Matrix4f model_matrix = model_control.model_matrix();
});
```
