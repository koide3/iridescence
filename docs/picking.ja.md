# ポイントピッキング

## クリックされた位置の取得

```cpp
viewer->register_ui_callback("ui", [&] {
  auto& io = ImGui::GetIO();
  // GLキャンバスを右クリックした場合
  if (!io.WantCaptureMouse && io.MouseClicked[ImGuiMouseButton_Right]) {
    // クリックされたピクセルの深度を取得
    float depth = viewer->pick_depth({io.MousePos.x, io.MousePos.y});

    // depth < 1.0f の場合、クリックされたピクセルは前景オブジェクト。それ以外の場合は背景。
    if(depth < 1.0f) {
      // クリックされたピクセルの3D位置を計算
      Eigen::Vector3f pos = viewer->unproject({io.MousePos.x, io.MousePos.y}, depth);
      viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::FlatRed().translate(pos).scale(0.05));
    }
  }
});
```

![picking](https://user-images.githubusercontent.com/31344317/210159144-bacd830b-aad9-4167-9a2d-c233efc4a0ff.gif)

クリックされたポイントが有効な3D座標を持っているかどうかを確認するユーティリティメソッド```pick_point()```もあります。
```cpp
viewer->register_ui_callback("ui", [&] {
  int button = ImGuiMouseButton_Right;
  std::optional<Eigen::Vector3f> pt = viewer->pick_point(button);
  if (pt) {
    std::cout << "clicked point=" << pt->transpose() << std::endl;
  }
}
```


## クリックされたオブジェクト情報の取得

```cpp
// Infoバッファを有効にする。
// Infoバッファは各ピクセルに4つのint値を格納できる追加のレンダリングターゲットです。
viewer->enable_info_buffer();

// info_values == {1, 0, 0, 0} と {2, 0, 0, 0} を持つ緑と青の立方体を描画
viewer->update_drawable("cube1", glk::Primitives::cube(),
  guik::FlatGreen().add("info_values", Eigen::Vector4i(1, 0, 0, 0)));
viewer->update_drawable("cube2", glk::Primitives::cube(),
  guik::FlatBlue().add("info_values", Eigen::Vector4i(2, 0, 0, 0)).translate({2.0, 0.0, 0.0}));

viewer->register_ui_callback("ui", [&] {
  auto& io = ImGui::GetIO();
  if (!io.WantCaptureMouse && io.MouseClicked[ImGuiMouseButton_Right]) {
    float depth = viewer->pick_depth({io.MousePos.x, io.MousePos.y});

    // クリックされたピクセルが前景オブジェクトの場合
    if (depth < 1.0f) {
      // クリックされたピクセルの info_values を取得
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
