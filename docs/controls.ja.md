# カメラとモデルの操作

## カメラ操作

```cpp
auto viewer = guik::LightViewer::instance();

// XY平面に沿って移動するカメラを使用 (LiDAR SLAMなどで一般的)
viewer->use_orbit_camera_control();

// XZ平面に沿って移動するカメラを使用 (Visual SLAMなどで一般的)
viewer->use_orbit_camera_control_xz();

// XY平面を見下ろすトップダウンカメラを使用 (2Dマップ表示などで一般的)
viewer->use_topdown_camera_control();

// 任意の姿勢をとれるアークボールカメラを使用
viewer->use_arcball_camera_control();

// キーボード(WASD)で操作するFPSカメラを使用
viewer->use_fps_camera_control();
```

動く物体にカメラの視点を合わせ続けるのに便利なメソッドとして ```lookat()``` があります。これは指定した位置がカメラビューの中心に来るようにカメラを移動させます。

```cpp
Eigen::Vector3f center_pos = ...;
viewer->lookat(center_pos);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/TarRKF_Xd2E?start=13" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
```lookat``` の例 (0:13 ~)

### キーボード操作

| キー                   | 説明                                           |
| ---------------------- | ---------------------------------------------- |
| Ctrl + 矢印キー        | カメラ移動                                     |
| Ctrl + Page UP / Down  | ズームイン/アウト                              |
| Ctrl + Home / End      | 移動速度の増加/減少 (継続)                   |
| Shift                  | 移動速度の増加 (押している間)                  |

### FPSカメラ操作

| キー                   | 説明                                           |
| ---------------------- | ---------------------------------------------- |
| W / A / S / D          | 前進 / 左移動 / 後退 / 右移動                  |
| R / F                  | 上昇 / 下降                                    |
| E / Q                  | 旋回 (ヨー回転)                                |

| マウス                 | 説明                                           |
| ---------------------- | ---------------------------------------------- |
| 左ボタン長押し         | ヨー / ピッチ回転                              |
| 右ボタン長押し         | 上昇 / 下降                                    |
| スクロールボタン長押し | 前進 / 後退 / 左移動 / 右移動                  |
| スクロール             | FOV変更                                        |


## カスタムカメラの実装

カメラのプロパティを直接制御したい場合は、`StaticCameraControl`と`StaticProjectionControl`を使用してください。例えばカメラの姿勢とプロジェクションを直接操作することで、AR描画が可能になります。

```cpp
Eigen::Isometry3f T_world_camera = ...;   // カメラ姿勢 (X = 右, Y = 下, Z = 前)
auto static_camera = std::make_shared<guik::StaticCameraControl>(T_world_camera);
viewer->set_camera_control(static_camera);

Eigen::Vector2i canvas_size = viewer->canvas_size();
Eigen::Matrix3f camera_matrix = ...;  // OpenCVカメラ内部行列 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
auto static_projection = std::make_shared<guik::StaticProjectionControl>(canvas_size, camera_matrix);
viewer->set_projection_control(static_projection);
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/BI42gZO-IEY?si=fFFv46eZN9ge-Y3d" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


## モデル行列の操作 (ImGuizmo)

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

注意: ImGuizmoは1つのフレーム内で2回以上描画することはできません。

![guizmo](https://user-images.githubusercontent.com/31344317/210159001-58b69d32-70b2-4fd1-9885-d40af93514d4.gif)

Guizmoはサブビューワ上に描画することもできます。

```cpp
auto sub = viewer->sub_viewer("sub");

auto model_control = std::make_shared<guik::ModelControl>("model_control");
sub->register_ui_callback("model_control", [=] {
  const ImVec2 canvas_rect_min = ImGui::GetItemRectMin();
  const ImVec2 canvas_rect_max = ImGui::GetItemRectMax();
  const int win_x = canvas_rect_min.x;
  const int win_y = canvas_rect_min.y;
  const int win_w = canvas_rect_max.x - canvas_rect_min.x;
  const int win_h = canvas_rect_max.y - canvas_rect_min.y;

  const Eigen::Matrix4f view_matrix = sub->get_camera_control()->view_matrix();
  const Eigen::Matrix4f projection_matrix = sub->get_projection_control()->projection_matrix();

  model_control->draw_gizmo(win_x, win_y, win_w, win_h, view_matrix, projection_matrix, true);

  sub->update_coord("coord", guik::VertexColor(model_control->model_matrix()));
});
```
