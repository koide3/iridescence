# はじめに

![Iridescence](assets/logo.png)

**Iridescence** は3Dアルゴリズム＆アプリケーション開発のための可視化ライブラリです。特に点群を中心とした三次元構造を手軽に扱えるように設計されており、個人レベルの研究開発プロジェクトで使用するような簡便なユーザインタフェースを素早く構築することに主眼を置いており、著者の研究業務や複数のオープンソースプロジェクトで利用しています[[1]](https://github.com/koide3/glim)[[2]](https://github.com/koide3/direct_visual_lidar_calibration)。

**[Documentation (en)](https://koide3.github.io/iridescence/)**, **[Documentation (日本語)](https://koide3.github.io/iridescence/ja)**, **[API(C++)](https://koide3.github.io/iridescence/doc_cpp/), [API(Python)](https://koide3.github.io/iridescence/doc_py/)**

[![ppa](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fapi.launchpad.net%2F1.0%2F~koide3%2F%2Barchive%2Fubuntu%2Firidescence%3Fws.op%3DgetPublishedSources%26status%3DPublished&query=entries.0.source_package_version&label=ppa)](https://launchpad.net/~koide3/+archive/ubuntu/iridescence) [![PyPI - Version](https://img.shields.io/pypi/v/pyridescence)](https://pypi.org/project/pyridescence/) [![Build](https://github.com/koide3/iridescence/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/iridescence/actions/workflows/build.yml) on Ubuntu 22.04 / 24.04 and Windows

## 特徴

- このライブラリが提供する機能:
    - 軽量・簡便な三次元可視化機能。点群を中心とした各種の三次元形状レンダリングに対応。
    - ラピッドプロトタイピングのためのUI機能 ([Dear ImGui](https://github.com/ocornut/imgui)統合)
- このライブラリが提供しない機能:
    - リアリスティックなレンダリングやシェーディング
    - リッチな3Dメッシュレンダリング (簡素なテクスチャ付きメッシュレンダリング機能は有り)

## 依存ライブラリ

- [GLFW](https://www.glfw.org/) ([zlib/libpng license](https://www.glfw.org/license.html))
- [gl3w](https://github.com/skaslev/gl3w) ([Public domain](https://github.com/skaslev/gl3w/blob/master/UNLICENSE))
- [Dear ImGui](https://github.com/ocornut/imgui) ([MIT license](https://github.com/ocornut/imgui/blob/master/LICENSE.txt))
- [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo) ([MIT license](https://github.com/CedricGuillemet/ImGuizmo/blob/master/LICENSE))
- [implot](https://github.com/epezent/implot) ([MIT license](https://github.com/epezent/implot/blob/master/LICENSE))
- [Eigen](https://eigen.tuxfamily.org/index.php) ([MPL2 license](https://www.mozilla.org/en-US/MPL/2.0/))
- [rapidhash](https://github.com/Nicoshev/rapidhash) ([MIT license](https://github.com/Nicoshev/rapidhash/blob/master/LICENSE))
- [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs) ([WTFPL license](https://github.com/samhocevar/portable-file-dialogs/blob/main/COPYING))

## インストール

### C++ : [PPA](https://launchpad.net/~koide3/+archive/ubuntu/iridescence) からインストール (Ubuntu)

```bash
# Install from PPA
sudo add-apt-repository -y ppa:koide3/iridescence
sudo apt install libiridescence-dev
```

### Python : [PyPI](https://pypi.org/project/pyridescence/) からインストール (Ubuntu and Windows)

注意：現在、CPython 3.14向けには後述のソースインストールが必要です。

```bash
# Install from PyPI
pip install pyridescence
```

<details>
  <summary>ソースインストール</summary>


### C++ : ソースインストール (Ubuntu)

```bash
# Install dependencies
sudo apt-get install -y libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev

# Build and install Iridescence
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake ..
make -j
sudo make install
```

### Python : ソースインストール (Ubuntu and Windows)

```bash
git clone https://github.com/koide3/iridescence --recursive
cd iridescence
pip install .
```

</details>

## 使用方法

### C++ : cmakeプロジェクトでIridescenceを使う

```cmake
# パッケージを検索
find_package(Iridescence REQUIRED)

# Includeディレクトリ設定とライブラリのリンク
add_executable(your_program
  src/your_program.cpp
)
target_link_libraries(your_program
  Iridescence::Iridescence
)
```


### C++ : 最小サンプルプログラム

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  // ビューワインスタンスを生成。
  // このインスタンスはプログラム全体を通して共有される。(シングルトンパターン)
  auto viewer = guik::LightViewer::instance();

  float angle = 0.0f;

  // UIを実装するため、UIレンダリングにコールバックを登録する。
  // ビューワがUIをレンダリングする際に、このコールバックが呼び出される。
  viewer->register_ui_callback("ui", [&]() {
    // UIコールバック内では、ImGuiコマンドを呼び出して自由にUIを構築できる。
    // ここでは"angle"を操作するための"DragFloat"と、ビューワを閉じるための
    // "Button"を作成している。
    ImGui::DragFloat("Angle", &angle, 0.01f);

    // "Close"ボタンが押されたら、ビューワを閉じる。
    if (ImGui::Button("Close")) {
      viewer->close();
    }
  });

  // spin_once()はビューワを1フレーム更新する。
  // ビューワが閉じられるとfalseを返すので、これを利用してループを回す。
  while (viewer->spin_once()) {
    // Iridescenceではレンダリングされるオブジェクトは"drawables"と呼ばれ、一意な名前で管理される。
    // ビューワにオブジェクトを登録する際には、色や形状を設定するShaderSettingを同時に登録する。
    // 
    // ここでは、以下の2つのオブジェクトを登録している:
    //   - 名前:sphere、オブジェクト:ソリッド単位球、設定：Rainbow(Z座標エンコード)＋回転変換
    //   - 名前:wire_sphere、オブジェクト:ワイヤフレーム単位球、設定：FlatColor(単色)＋回転変換
    Eigen::AngleAxisf transform(angle, Eigen::Vector3f::UnitZ());
    viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow(transform));
    viewer->update_drawable("wire_sphere", glk::Primitives::wire_sphere(), guik::FlatColor({0.1f, 0.7f, 1.0f, 1.0f}, transform));
  }

  return 0;
}
```

### Python : 最小サンプルプログラム

```py
#!/bin/env python
import numpy
from scipy.spatial.transform import Rotation

from pyridescence import *

# ビューワインスタンスを生成。
# このインスタンスはプログラム全体を通して共有される。(シングルトンパターン)
viewer = guik.LightViewer.instance()

angle = 0.0

# UIを定義するコールバック関数。
# ビューワがUIをレンダリングする際に、このコールバックが呼び出される。
def ui_callback():
  # UIコールバック内では、ImGuiコマンドを呼び出して自由にUIを構築できる。
  # ここでは"angle"を操作するための"DragFloat"と、ビューワを閉じるための
  # "Button"を作成している。
  global angle
  updated, angle = imgui.drag_float('angle', angle, 0.01)

  # "Close"ボタンが押されたら、ビューワを閉じる。
  if imgui.button('close'):
    viewer.close()

# UIコールバックをビューワに登録する。
viewer.register_ui_callback('ui', ui_callback)

# spin_once()はビューワを1フレーム更新する。
# ビューワが閉じられるとfalseを返すので、これを利用してループを回す。
while viewer.spin_once():
  # Iridescenceではレンダリングされるオブジェクトは"drawables"と呼ばれ、一意な名前で管理される。
  # ビューワにオブジェクトを登録する際には、色や形状を設定するShaderSettingを同時に登録する。
  # 
  # ここでは、以下の2つのオブジェクトを登録している:
  #   - 名前:sphere、オブジェクト:ソリッド単位球、設定：Rainbow(Z座標エンコード)＋回転変換
  #   - 名前:wire_sphere、オブジェクト:ワイヤフレーム単位球、設定：FlatColor(単色)＋回転変換
  transform = numpy.identity(4)
  transform[:3, :3] = Rotation.from_rotvec([0.0, 0.0, angle]).as_matrix()
  viewer.update_drawable('sphere', glk.primitives.sphere(), guik.Rainbow(transform))
  viewer.update_drawable('wire_sphere', glk.primitives.wire_sphere(), guik.FlatColor(0.1, 0.7, 1.0, 1.0, transform))

```

![example_01](https://user-images.githubusercontent.com/31344317/210127177-31630466-f8a1-45b6-8bc7-2fdd2e4c9548.gif)

詳細は **[documentation](https://koide3.github.io/iridescence/)** を参照してください。

## 使用例

![ral2021](https://user-images.githubusercontent.com/31344317/210128637-80f79abf-69c3-479c-91e9-0807e5b8b3ae.jpg)
![iros2022](https://user-images.githubusercontent.com/31344317/210128635-2ef02dff-3d74-499e-bde8-2c9c0dc047ff.jpg)

## ライセンス

IridescenceはMITライセンスで公開されています。

## 余談

[開発経緯など](yodan)