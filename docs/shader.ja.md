# シェーダ設定

## シェーダ設定

**guik::ShaderSetting** クラスは、割り当てられたオブジェクトのレンダリングを制御するための、レンダリングパラメータ（例：カラーモードやモデル行列）を保持します。これを使用して、オブジェクトの姿勢・形状・色を設定することができます。

```cpp
// 球オブジェクト。
std::shared_ptr<glk::Drawable> drawable = glk::Primitives::sphere();

// guik::ShaderSetting はカラーモードや姿勢・形状を表すモデル行列を保持する。
int color_mode = glk::ColorMode::RAINBOW;
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
auto shader_setting = guik::ShaderSetting(color_mode, transformation)

// オブジェクトとシェーダ設定のペアをビューワに登録する。
// 設定されたパラメータに基づいてオブジェクトがレンダリングされる。
viewer->update_drawable("drawable_name", drawable, shader_setting);
```

## モデル変換行列

オブジェクトモデルに適用される姿勢・形状変換行列をモデル変換行列といいます。**guik::ShaderSetting**ではEigenの変換行列（例：```Eigen::Matrix4f```, ```Eigen::Isometry3f```, ```Eigen::Affine3f```およびそれらのdouble版）を使ってモデル行列を設定できます。

また、**guik::ShaderSetting**はモデル行列を操作するためのユーティリティメソッドも備えています。ユーティリティメソッドは元のモデル変換行列の右辺から変換を適用します。

```cpp
// モデル変換行列は Identity() * Rotation(3.14rad, (1,0,0)) * Translation(1,0,0) * Scale(0.1) になる。
auto setting = guik::Rainbow().rotate(3.14f, {1.0f, 0.0f, 0.0f}).translate({1.0f, 0.0f, 0.0f}).scale(0.1f)
viewer->update_drawable("drawable_name", drawable, setting);
```

## カラーモード

Iridescenceでは5つのカラーモードが用意されており、それぞれに対応する**guik::ShaderSetting**派生のユーティリティクラスが用意されています:

- **RAINBOW (guik::Rainbow)** モードでは、各ピクセルの3D位置をエンコードした色でオブジェクトを描画します（デフォルトでは各ピクセルの高さ（z座標）をエンコードします。`set_rainbow_range()`と`set_rainbow_axis()`で設定変更可能）。
- **FLAT_COLOR (guik::FlatColor)** モードでは、単一色でオブジェクトを描画します。
- **VERTEX_COLOR (guik::VertexColor)** モードでは頂点ごとに設定された色を補間してオブジェクトを描画します。　
- **TEXTURE_COLOR (guik::TextureColor)** モードではテクスチャからピクセル色をサンプリングしてオブジェクトを描画します。
- **VERTEX_COLORMAP (guik::VertexColorMap)** モードでは頂点に設定されたカラーマップ値をエンコードした色でオブジェクトを描画します（RAINBOWに似ていますが、反射強度などの任意の頂点スカラー属性を使用できます）。

![Screenshot_20230101_004203](https://user-images.githubusercontent.com/31344317/210148371-c12e7126-2dc2-48e5-b43b-b57a7be9d92e.png)
Left to right: Rainbow, FlatColor, VertexColor, TextureColor (transparent)


```cpp
Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

// RAINBOW
auto shader_setting = guik::Rainbow(transformation);

// FLAT_COLOR
Eigen::Vector4f color(1.0f, 0.5f, 0.0f, 1.0f);
auto shader_setting = guik::FlatColor(color, transformation);

// 使用頻度の高い色のためのユーティリティクラスも有ります。
// guik::FlatRed() == guik::FlatColor({1.0f, 0.0f, 0.0f, 1.0f});
// guik::FlatGreen() == guik::FlatColor({0.0f, 1.0f, 0.0f, 1.0f});
// guik::FlatBlue() == guik::FlatColor({0.0f, 0.0f, 1.0f, 1.0f});
// guik::FlatOrange() == guik::FlatColor({1.0f, 0.5f, 0.0f, 1.0f});

// VERTEX_COLOR
auto shader_setting = guik::VertexColor(transformation);

// TEXTURE_COLOR 透明度有り
auto shader_setting = guik::TextureColor(transformation).make_transparent();

// VERTEX_COLORMAP
auto shader_setting = guik::VertexColorMap(transformation);
```

![Screenshot_20230101_005425](https://user-images.githubusercontent.com/31344317/210149282-38377bad-dfb8-4f86-a907-60cdcef10b92.png)
RAINBOWモードでレンダリングされた点群。点の高さに基づいて色が付けられている。
