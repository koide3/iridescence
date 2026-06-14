# クックブック

## 点群エディタ

[src/example/ext_pointcloud_editor.cpp](https://github.com/koide3/iridescence/blob/master/src/example/ext_pointcloud_editor.cpp)

<details>
  <summary>ext_pointcloud_editor.cpp</summary>
```cpp linenums="1"
#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <portable-file-dialogs.h>

class PointCloudEditor {
public:
  PointCloudEditor() {
    auto viewer = guik::LightViewer::instance();

    cube_matrix.reset(new guik::ModelControl("cube_matrix"));

    viewer->register_ui_callback("ui", [this] { ui_callback(); });
    viewer->spin();
  }

private:
  void ui_callback() {
    auto viewer = guik::LightViewer::instance();

    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // 点群の読み込み    
    if (ImGui::Button("Load point cloud")) {
      for (const auto& filename : pfd::open_file("Select a PLY file").result()) {
        // PLYから点群を読み込み、点リストに追加
        auto ply = glk::load_ply(filename);
        if (ply) {
          points.insert(points.end(), ply->vertices.begin(), ply->vertices.end());
        }
      }

      // ビューワに点を表示
      if (!points.empty()) {
        cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
        viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      }
    }

    // 点群の保存
    if (ImGui::Button("Save point cloud")) {
      auto path = pfd::save_file("Select a destination path to save PLY").result();
      if (!path.empty() && !points.empty()) {
        glk::save_ply_binary(path, points.data(), points.size());
      }
    }

    // フィルタリング領域を表す立方体を表示
    ImGui::Separator();
    cube_matrix->draw_gizmo_ui();
    cube_matrix->draw_gizmo();
    viewer->update_drawable("cube", glk::Primitives::cube(), guik::FlatColor({1.0f, 0.5f, 0.0f, 0.5f}, cube_matrix->model_matrix()).make_transparent());

    // フィルタリング領域内の点を検索
    if (ImGui::Button("Select points")) {
      // 立方体のモデル行列の逆行列は、ワールド座標系の点を立方体座標系に変換します
      Eigen::Matrix4f inv_cube_matrix = cube_matrix->model_matrix().inverse();

      selected_points.clear();
      neg_selected_points.clear();
      for (int i = 0; i < points.size(); i++) {
        Eigen::Vector3f pt = (Eigen::Affine3f(inv_cube_matrix) * points[i]);
        // [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)] の範囲にある点は立方体の内部にあります
        if ((pt.array() > Eigen::Array3f::Constant(-0.5f)).all() && (pt.array() < Eigen::Array3f::Constant(0.5f)).all()) {
          selected_points.emplace_back(i);
        } else {
          neg_selected_points.emplace_back(i);
        }
      }

      // 選択された点を大きなオレンジ色の点で表示
      viewer->update_drawable("selected", std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, selected_points), guik::FlatOrange().set_point_scale(2.0f));
    }

    // 選択された点を点リストから削除
    if (ImGui::Button("Remove points")) {
      // 選択されていない点のみを残す
      std::vector<Eigen::Vector3f> filtered;
      std::transform(neg_selected_points.begin(), neg_selected_points.end(), std::back_inserter(filtered), [&](const auto i) { return points[i]; });
      points = std::move(filtered);

      selected_points.clear();
      neg_selected_points.clear();

      cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
      viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
      viewer->remove_drawable("selected");
    }

    ImGui::End();
  }

private:
  std::unique_ptr<guik::ModelControl> cube_matrix;      // フィルタリング領域を表す立方体のモデル行列

  std::vector<Eigen::Vector3f> points;                  // 点群
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;  // 点のCloudBuffer
  std::vector<unsigned int> selected_points;            // 選択された点
  std::vector<unsigned int> neg_selected_points;        // 選択されていない点
};

int main(int argc, char** argv) {
  PointCloudEditor editor;
  return 0;
}
```
</details>

- portable-file-dialogsを使用したPLYファイルの入出力
- Gizmoベースのモデル行列操作
- ```PointCloudBuffer``` と ```IndexedPointCloudBuffer``` を使用した点群レンダリング

<iframe width="560" height="315" src="https://www.youtube.com/embed/pHPGS8VNN2o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
