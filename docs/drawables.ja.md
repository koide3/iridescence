# 描画オブジェクト

描画可能な線、3Dプリミティブ、2Dプリミティブの例 ([Code](https://github.com/koide3/iridescence/blob/master/src/example/02_light_viewer_primitives.cpp)):
![primitives](https://user-images.githubusercontent.com/31344317/210129208-2d126725-67d9-48ff-9eae-7af118a319f9.png)


## ショートハンドメソッド


頻繁に使用されるオブジェクトタイプのために、```guik::LightViewer``` に略記メソッドを複数用意しています。これらのメソッドは、オブジェクトの作成と ```update_drawable()``` の呼び出しを1つの関数で行います。

```cpp
// 3Dプリミティブ
viewer->update_sphere("sphere", guik::FlatRed());
viewer->update_wire_sphere("wire_sphere", guik::FlatRed());
viewer->update_coord("coord", guik::VertexColor());
viewer->update_wire_frustum("frustum", guik::FlatGreen());

// 点群
// 各点をVector(3|4)(f|d)とした生配列やstd::vectorから点群オブジェクトを作成・登録できます。
std::vector<Eigen::Vector4d> points = ...;
viewer->update_points("points", points, guik::Rainbow());

// 正規分布（点群＋共分散行列）
std::vector<Eigen::Vector3f> means = ...;
std::vector<Eigen::Matrix3f> covs = ...;
float scale = 1.0f;
viewer->update_normal_dists("normal_dists", means, covs, scale, guik::Rainbow());

// 線集合（line_strip=trueの場合、頂点間を数珠つなぎした線を描画します）
std::vector<Eigen::Vector3f> line_vertices = ...;
bool line_strip = true;
viewer->update_thin_lines("lines", line_vertices, true, guik::FlatGreen());
```


## 3Dプリミティブ

以下のプリミティブが **glk::Primitives** から利用可能です。
- Icosahedron (正二十面体)
- Sphere (球)
- Stanford bunny (スタンフォードバニー)
- Cube (立方体)
- Cone (円錐)
- Coordinate system (座標系を表す線)
- Frustum (視錐台)

```cpp
#include <glk/primitives/primitives.hpp>

// ソリッドおよびワイヤーフレームの正二十面体
glk::Primitives::icosahedron();
glk::Primitives::wire_icosahedron();

// ソリッドおよびワイヤーフレームの球
glk::Primitives::sphere();
glk::Primitives::wire_sphere();

// ソリッドおよびワイヤーフレームのバニー
glk::Primitives::bunny();
glk::Primitives::wire_bunny();

// ソリッドおよびワイヤーフレームの立方体
glk::Primitives::cube();
glk::Primitives::wire_cube();

// ソリッドおよびワイヤーフレームの円錐
glk::Primitives::cone();
glk::Primitives::wire_cone();

// RGBカラーの座標系を表す線
// 色付けのために guik::VertexColor でレンダリングする必要があります
glk::Primitives::coordinate_system();
glk::Primitives::solid_coordinate_system();

// カメラ姿勢を表すワイヤーフレーム視錐台 (+Z=前方)
glk::Primitives::wire_frustum();
```

![Screenshot_20221231_182844](https://user-images.githubusercontent.com/31344317/210131821-42071de7-3ace-433b-9cb4-d39d9444ee85.png)


## 線

**glk::ThinLines** は線の太さが一定の線を描画します。

```cpp
#include <glk/thin_lines.hpp>

// 線の頂点
std::vector<Eigen::Vector3f> vertices = ...;

// line_strip == true の場合、隣接する頂点間に線が描画されます (GL_LINE_STRIP)。
// line_strip == false の場合、vertices[i * 2] と vertices[i * 2 + 1] の間に線が描画されます (GL_LINES)。
// 参考 : https://tkengo.github.io/assets/img/2015-01-03-opengl-es-2-2d-knowledge-2/gl-lines.png
bool line_strip = true;

// 線オブジェクトの生成 (すべての頂点が順番に利用されます)
auto lines = std::make_shared<glk::ThinLines>(vertices, line_strip);

// 頂点の順序を指定して線オブジェクトを作成
std::vector<unsigned int> indices = ...;
auto lines_with_indices = std::make_shared<glk::ThinLines>(vertices, indices, line_strip);

// 頂点カラー付きの線オブジェクトの作成
std::vector<Eigen::Vector4f> colors = ...;
auto lines_with_colors = std::make_shared<glk::ThinLines>(vertices, colors, line_strip);

// 幅の設定 (glLineWidth)
lines->set_line_width(2.0f);
```


**glk::Lines** はポリゴンで線を描画します。線の太さは視点と視野角に応じて変化します。
```cpp
#include <glk/thin_lines.hpp>

float line_width = 0.1f;
std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector4f> colors = ...;
bool line_strip = true;

auto lines = std::make_shared<glk::Lines>(line_width, vertices, colors, line_strip);
```

![Screenshot_20221231_183450](https://user-images.githubusercontent.com/31344317/210131978-1b99b57e-193b-4196-8887-fffe51a858c6.png)


## 点群

**glk::PointCloudBuffer** は3D点群をレンダリングします。

```cpp
#include <glk/pointcloud_buffer.hpp>

// std::vector<Eigen::Vector3f> から PointCloudBuffer を作成
std::vector<Eigen::Vector3f> vertices = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);

// 各点の色を追加
std::vector<Eigen::Vector4f> colors = ...;
cloud_buffer->add_color(colors);

// [0, 1] のスカラー値をエンコードする頂点カラーの追加 (VERTEX_COLORモードで参照される)
std::vector<double> intensities = ...;
cloud_buffer->add_intensity(glk::COLORMAP::TURBO, intensities);

// [0, 1] のスカラー値をカラーマップ属性として追加 (VERTEX_COLORMAPモードで参照される)
cloud_buffer->add_colormap(intensities);

// 頂点法線の追加
std::vector<Eigen::Vector3f> normals = ...;
cloud_buffer->add_normals(normals);

// 各点に任意属性データを追加
std::vector<float> values = ...;
int dim = 1;
cloud_buffer->add_buffer("radius", dim, values.data(), sizeof(float) * dim, values.size());

// 点の描画サイズを設定
auto shader_setting = guik::Rainbow().set_point_scale(2.0f);
viewer->update_drawable("points", cloud_buffer, shader_setting);
```

![Screenshot_20230101_005425](https://user-images.githubusercontent.com/31344317/210149282-38377bad-dfb8-4f86-a907-60cdcef10b92.png)
```guik::Rainbow``` でレンダリングされた ```glk::PointCloudBuffer```

**glk::IndexedPointCloudBuffer** ではレンダリングする頂点のインデックスを指定できます。

```cpp
#include <glk/indexed_pointcloud_buffer.hpp>

std::vector<Eigen::Vector3f> vertices = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);

std::vector<unsigned int> indices = ...;
auto indexed_buffer = std::make_shared<glk::IndexedPointCloudBuffer>(cloud_buffer, indices);
```

### 点の形状

デフォルトの点のサイズ・形状は `set_point_shape()` メソッドで設定できます。

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();

float point_size = 0.01f;  // 点のベースサイズ
bool metric = true;        // Trueの場合、三次元空間スケール、Falseの場合、スクリーンスペーススケールで点を描画
bool circle = true;        // Trueの場合、点を円形で描画、Falseの場合、点を四角形で描画（少し高速）
global_setting.set_point_shape(0.01f, true, true);
```

### 点のスケール

**メトリックスペーススケーリング（デフォルト）**  
各点は `radius_m = point_scale * point_size + point_size_offset` で計算される三次元空間における大きさで描画されます。

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();
global_setting.set_point_shape_circle();
global_setting.set_point_scale_metric();
global_setting.set_point_size(0.5f);  // デフォルトの点半径を 0.5 に設定

auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(...);
// 点半径を 0.5 に設定
viewer->update_drawable("points", cloud_buffer, guik::FlatBlue().set_point_size(0.5f));
```

![ss_1710604595 779500](https://github.com/koide3/iridescence/assets/31344317/f13f8c91-8dcc-4cc0-8dfa-9e4edcb2f518)
赤 : ワイヤーフレーム球 (半径=0.5), 青 : PointCloudBuffer でレンダリングされた点 (`point_shape_mode=CIRCLE`, `point_scale_mode=METRIC`, `point_size=0.5`).

<details>
<summary>スクリーンスペーススケーリング（旧実装）</summary>

点のサイズは `radius_pix = point_scale * point_size * nz + point_size_offset` として計算されます。ここで `nz` は [0, 1] のピクセルのスクリーンスペース深度です。デフォルトでは `point_scale=1.0`, `point_size=10.0`, `point_size_offset=0.0` であり、これらは `guik::ShaderSetting` に値を設定することで更新できます。

```cpp
auto viewer = guik::viewer();
guik::ShaderSetting& global_setting = viewer->shader_setting();
  global_setting.set_point_scale_screenspace();   // 点スケールモードをスクリーンスペースに設定
  global_setting.set_point_size(5.0f);            // 基本点サイズを 5.0 に設定

auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(...);
// 点のサイズをベースサイズの2倍にする
viewer->update_drawable("points", cloud_buffer, guik::FlatBlue().set_point_scale(2.0f));
```

</details>

## 正規分布集合

**glk::NormalDistributions**

```cpp
#include <glk/normal_distributions.hpp>

std::vector<Eigen::Vector3f> means = ...;
std::vector<Eigen::Matrix3f> covs = ...;
float scale = 1.0f;

auto normal_distributions = std::make_shared<glk::NormalDistributions>(means, covs, scale);
```

## ポイントスプラッティング

**glk::Splatting** は点群を向きのある円盤の集合としてレンダリングします。

```cpp
#include <glk/splatting.hpp>

// 法線付きの PointCloudBuffer を作成
std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector3f> normals = ...;
auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertices);
cloud_buffer->add_normals(normals);

// スプラッティングシェーダーを作成
auto splatting_shader = glk::create_splatting_shader();

// スプラッティングインスタンスを作成
float point_radius = 0.1f;
auto splatting = std::make_shared<glk::Splatting>(splatting_shader);
splatting->set_point_radius(point_radius);
splatting->set_cloud_buffer(cloud_buffer);

// 頂点半径が有効な場合、各点の半径は point_radius * 頂点の半径 として計算されます。
// それ以外の場合、固定の point_radius がすべての点のレンダリングに使用されます。
splatting->enable_vertex_radius();

std::vector<float> radii = ...;
cloud_buffer->add_buffer("radius", 1, radii.data(), sizeof(float), radii.size());
```

![points](https://user-images.githubusercontent.com/31344317/210149278-ac7a1424-5846-4a8c-94a8-dc2173229566.png)
疎な点群

![splats](https://user-images.githubusercontent.com/31344317/210149280-7160f17f-c6cd-46c5-a66c-4a41d480db69.png)
```glk::Splatting``` を使用してレンダリングされた点群

![splats2](https://user-images.githubusercontent.com/31344317/210149281-18ad2296-4bc6-4f44-9ef5-0696f1b03141.png)
結果図: 各点は向きを持った円盤としてレンダリングされます


## メッシュ

```cpp
#include <glk/mesh.hpp>

std::vector<Eigen::Vector3f> vertices = ...;
std::vector<Eigen::Vector3f> normals = ...;
std::vector<Eigen::Vector4f> colors = ...;
std::vector<Eigen::Vector2f> tex_coords;
std::vector<unsigned int> indices;

// メッシュインスタンスを作成
// normal/color/tex_coord が無い場合は nullptr を渡します
auto mesh = std::make_shared<glk::Mesh>(
  vertices.data(), sizeof(float) * 3,
  normals.data(), sizeof(float) * 3,
  colors.data(), sizeof(float) * 4,
  tex_coords.data(), sizeof(float) * 2,
  vertices.size()
  indices.data(),
  indices.size()
);

std::shared_ptr<glk::Texture> texture = ...;
mesh->set_texture(texture);
```

## 2Dオブジェクト

**guik::HoveredDrawings** は三次元座標位置を画面に投影し、投影された画面上位置に2Dプリミティブを描画します。

```cpp
#include <guik/hovered_drawings.hpp>

// 2D描画レンダラーを作成し、ビューワに登録
auto hovered = std::make_shared<guik::HoveredDrawings>();
viewer->register_ui_callback("hovered", hovered->create_callback());

// 固定の三次元座標 (1.0, 2.0, 3.0) にテキストを描画
std::uint32_t fg_color = IM_COL32(255, 255, 255, 255);
std::uint32_t bg_color = IM_COL32(0, 0, 0, 128);
hovered->add_text({1.0f, 2.0f, 3.0f}, "text1", fg_color, bg_color);

// 三次元座標を直接指定する代わりに3Dオブジェクト名を使用して、
// そのオブジェクトにオーバーレイして2D描画を行うこともできます
hovered->add_text_on("drawable_name", "text2", fg_color, bg_color);


// クロス
Eigen::Vector3f position = {1.0f, 2.0f, 3.0f};
std::uint32_t color = IM_COL32(255, 255, 255, 255);
float size = 10.0f;
hovered->add_cross(position, color, size);

// 円
float radius = 10.0f;
hovered->add_circle(position, color, radius);

// 三角形
float height = 20.0f;
hovered->add_triangle(position, color, height);
hovered->add_filled_triangle(position, color, height);

// 矩形
Eigen::Vector2f size = {10.0f, 10.0f};
Eigen::Vector2f offset = {0.0f, 0.0f};
hovered->add_rect(position, color, size, offset);
hovered->add_filled_rect(position, color, size, offset);

// 画像 (glk::Texture)
std::make_shared<glk::Texture> texture = ...;
hovered->add_image(position, texture, size, offset);
```

![Screenshot_20221231_183315](https://user-images.githubusercontent.com/31344317/210131927-75c87acf-a85d-4c8c-a877-1ae8d18e15ad.png)


**guik::HoveredDrawings** はサブビューワ上に描画することもできます。

```cpp
auto sub = viewer->sub_viewer("sub");
auto hovered = std::make_shared<guik::HoveredDrawings>(sub);
hovered->add_rect_on("coord", IM_COL32(0, 255, 0, 255));
sub->register_ui_callback("hovered", hovered->create_callback());
```


## 画像 (2Dテクスチャ)

```cpp
#include <glk/texture.hpp>

// 生のピクセルデータからテクスチャを作成
Eigen::Vector2i size = ...;
GLuint internal_format = GL_RGBA;
GLuint format = GL_RGB;
GLuint type = GL_UNSIGNED_BYTE;
std::vector<unsigned char> pixels = ...;
auto texture = std::make_shared<glk::Texture>(size, internal_format, format, type, pixels.data());

// 画像をビューワに登録
viewer->update_image("image", texture);
```

```cv::Mat``` からテクスチャを作成するユーティリティ関数もあります。
```cpp
#include <glk/texture_opencv.hpp>

cv::Mat image = ...;
auto texture = glk::create_texture(image);

viewer->update_image("image", texture);
```

![Screenshot_20230101_011615](https://user-images.githubusercontent.com/31344317/210149508-e98dd695-9a38-4bd9-8216-96aa5c2510d1.png)

画像名に '/' が含まれている場合、スラッシュの前の文字列はグループ名として認識され、同じグループ名の画像はタブにグループ化されます。

```cpp
viewer->update_image("group0/image0", texture);
viewer->update_image("group0/image1", texture);
viewer->update_image("group1/image0", texture);
```

![Screenshot_20230101_011918](https://user-images.githubusercontent.com/31344317/210149509-c096fdcb-7337-44bf-833d-ce369378bbe1.png)


## プロット (ImPlot)

```cpp
#include <implot.h>
#include <guik/viewer/light_viewer.hpp>


std::vector<double> xs = ...;
std::vector<double> ys = ...;

// 基本的なプロット
viewer->setup_plot("curves_y", 1024, 256);
viewer->update_plot_line("curves_y", "sin", ys_sin);  // Y値のみが与えられた場合、X値はインデックスIDになります
viewer->update_plot_stairs("curves_y", "sin_stairs", ys_sin);


std::vector<double> xs_circle = ...;
std::vector<double> ys_circle = ...;

// プロット名に '/' が含まれている場合、スラッシュの前の文字列はグループ名として認識されます。
// 同じグループ名のプロットは同じタブに表示されます。
viewer->setup_plot("group02/circle", 1024, 1024, ImPlotFlags_Equal);
viewer->update_plot_line("group02/circle", "circle", xs_circle, ys_circle, ImPlotLineFlags_Loop);
```

![Screenshot_20231212_103830](https://github.com/koide3/iridescence/assets/31344317/0036c0ab-63bb-451f-8f0f-e8931c820f71)
