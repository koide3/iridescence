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

// Use an FPS-like camera control with keyboard (WASD) control
viewer->use_fps_camera_control();
```

A useful method to let the camera keep focusing on a moving object is ```lookat()``` that moves the camera such that the specified position comes to the center of the camera view.

```cpp
Eigen::Vector3f center_pos = ...;
viewer->lookat(center_pos);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/TarRKF_Xd2E?start=13" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
```lookat``` example (0:13 ~)

### Keyboard control

| Key                    | Description                                    |
| ---------------------- | ---------------------------------------------- |
| Ctrl + Arrow           | Move camera                                    |
| Ctrl + Page UP / Down  | Zoom in/out                                    |
| Ctrl + Home / End      | Increase / decrease moving speed (permanently) |
| Shift                  | Increase moving speed (while holding)          |

### FPS-camera control

| Key                    | Description                                    |
| ---------------------- | ---------------------------------------------- |
| W / A / S / D          | Forward / Left / Backward / Right              |
| R / F                  | Up / Down                                      |
| E / Q                  | Heading (Yaw rotation)                         |

| Mouse                  | Description                                    |
| ---------------------- | ---------------------------------------------- |
| Hold left button       | Yaw / Pitch rotation                           |
| Hold right button      | Up / Down                                      |
| Hold scroll button     | Forward / Backward / Left/ Right               |
| Scroll                 | Change FOV                                     |


## Implementing custom camera control

In case you want to directly control the camera properties, use `StaticCameraControl` and `StaticProjectionControl`. This  combination enables, for example, AR-like visualization through direct manipulation of camera pose and projection.

```cpp
Eigen::Isometry3f T_world_camera = ...;   // Camera pose (X = right, Y = down, Z = forward)
auto static_camera = std::make_shared<guik::StaticCameraControl>(T_world_camera);
viewer->set_camera_control(static_camera);

Eigen::Vector2i canvas_size = viewer->canvas_size();
Eigen::Matrix3f camera_matrix = ...;  // OpenCV camera intrinsic matrix [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
auto static_projection = std::make_shared<guik::StaticProjectionControl>(canvas_size, camera_matrix);
viewer->set_projection_control(static_projection);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/BI42gZO-IEY?si=fFFv46eZN9ge-Y3d" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


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
