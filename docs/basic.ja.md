# 基本操作

## ビューワインスタンスの生成

```guik::LightViewer::instance()``` はビューワインスタンスを生成し返します。ビューワインスタンスは最初の呼び出し時に生成され、それ以後の ```guik::LightViewer::instance()``` の結果は同じグローバルインスタンスを参照します（シングルトンパターン）。

```viewer->spin_once()``` ビューワの内容を更新し、3Dコンテンツを1フレームだけレンダリングします。ビューワウィンドウが閉じられると ```false``` を返します。

以下に空のビューワを表示する最小サンプルコードを示します。

```cpp
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  // ビューワインスタンスの作成
  auto viewer = guik::LightViewer::instance();

  // ウィンドウが閉じられるまで更新を続ける
  while (viewer->spin_once()) {}
}
```

![Screenshot_20230102_223722](https://user-images.githubusercontent.com/31344317/210238748-b4f463ee-af08-425c-86d3-7b21ae4b18e5.png)

```spin_once()``` の代わりに ```spin()``` を使って、ウィンドウが閉じられるまでビューワを回し続けることもできます。また、```guik::viewer()``` という ```guik::LightViewer::instance()``` と同等の略記関数も用意されています。

これらのユーティリティ関数を使って、最小サンプルコードをより短く書くこともできます（上記のサンプルと同じ処理）。

```cpp
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();
  viewer->spin();
}
```

## 3Dオブジェクト(Drawables)の登録

Iridescenceではレンダリングされる3Dオブジェクトは"drawables"と呼ばれ、一意な名前で管理されます。以下にワイヤフレーム球をビューワに登録するサンプルコードを示します。

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  // update_drawable でオブジェクトをビューワに登録できる。
  // 第一引数 : オブジェクトID (名前)
  // 第二引数 : 登録するオブジェクトインスタンス (Drawable)
  // 第三引数 : 色・姿勢・形状設定 (ShaderSetting)
  viewer->update_drawable(
    "sphere",
    glk::Primitives::wire_sphere(),
    guik::FlatRed()
  );

  viewer->spin();
}
```

![Screenshot_20230102_233458](https://user-images.githubusercontent.com/31344317/210245441-2506d0c1-e044-4ed3-904d-00f9c96ef520.png)

```update_drawable()```の第一引数は登録するオブジェクトに付与する名前（ID）です。すでに同じ名前で登録されているオブジェクトがある場合、新しいオブジェクトで上書きされます。

第二引数は登録するオブジェクトインスタンスです。サポートされている3Dオブジェクトの種類は[Drawables](drawables.md)を参照してください。

第三引数はレンダリングパラメータを保持するシェーダ設定です。シェーダ設定のパラメータを変更することで、描画オブジェクトの色、姿勢、形状を制御できます。以下に例を示します。詳細は[Shader setting](shader.md)を参照してください。

```cpp
auto transformation1 = Eigen::Translation3f(0.0f, -2.0f, 0.0f);
auto setting1 = guik::Rainbow(transformation1);
viewer->update_drawable("sphere1", glk::Primitives::wire_sphere(), setting1);

auto transformation2 = Eigen::Translation3f(0.0f, 2.0f, 0.0f);
auto setting2 = guik::FlatColor({1.0f, 0.5f, 0.2f, 1.0f}, transformation2);
viewer->update_drawable("sphere2", glk::Primitives::wire_sphere(), setting2);
```

![Screenshot_20230102_235102](https://user-images.githubusercontent.com/31344317/210247287-5f51f5c7-58a2-4c55-bed0-b1245bad6bde.png)


!!!tip
    ビューワクラスにはオブジェクト登録のための省略関数（例: ```update_sphere()```）も用意されており、```guik/viewer/light_viewer.hpp```をインクルードするだけで大体のオブジェクトを表示可能です。

    ```cpp
    // ビューワに赤い球を登録（ID="sphere"）
    viewer->update_sphere("sphere", guik::FlatRed());
    ```

!!!tip
    ShaderSettingクラスにもオブジェクト姿勢・形状指定のための省略関数（例: ```translate()```, ```rotate()```, ```scale()```）が用意されています。以下のコードは、平行移動とスケーリングを適用する例です。

    ```cpp
    // ビューワに平行移動とスケーリングを適用した赤い球を登録（ID="sphere"）
    viewer->update_sphere("sphere", guik::FlatRed().translate({1.0, 2.0, 3.0}).scale(0.1));
    ``` 


## UIコールバックの登録 (Dear ImGui)

研究開発現場ではたくさんのパラメータを試行錯誤して変更しながらシステムを作ることがままあります。設定ファイルなどからパラメータを読み込む方式などが広く使用されていますが、個人的な経験ではGUI上でインタラクティブにパラメータを変更できると、パラメータの影響を即座に確認できるため、開発効率が大幅に向上することが多いです。

Iridescenceでは手軽にインタラクティブなユーザインターフェースを設計・開発できるよう、即時モードGUIライブラリである[Dear ImGui](https://github.com/ocornut/imgui)との連携機能を提供しています。

GUIを作成するには、```register_ui_callback()```でUIレンダリングイベントへのコールバック関数を登録します。UIコールバック関数内ではImGUIのコマンドを呼び出して自由にUIを構築できます。以下に、球体を回転させるための```DragFloat```と、ビューワウィンドウを閉じるための```Button```を持つ簡単なGUIを作成する例を示します。

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
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

  while (viewer->spin_once()) {
    // "angle"に応じて球を回転させる
    Eigen::AngleAxisf transform(angle, Eigen::Vector3f::UnitZ());
    viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow(transform));
    viewer->update_drawable("wire_sphere", glk::Primitives::wire_sphere(), guik::FlatColor({0.1f, 0.7f, 1.0f, 1.0f}, transform));
  }

  return 0;
}
```

![example_01](https://user-images.githubusercontent.com/31344317/210127177-31630466-f8a1-45b6-8bc7-2fdd2e4c9548.gif)

使用できるUIを一覧したい場合はビューワメニュー(`Ctrl + M`で表示)から`Utility -> ImGui -> Demo window`を選択するとウィジェットデモを表示できます。デモで使用したいウィジェットを見つけ、その箇所の[コード](https://github.com/ocornut/imgui/blob/88dfd85e9296fc03ac858f66cc6507ba443294ef/imgui_demo.cpp)を見ることで利用方法を確認できます。その他詳細は [Dear ImGui](https://github.com/ocornut/imgui/blob/master/docs/FAQ.md#q-where-is-the-documentation) を参照してください。

!!!note 
    Dear ImGui に加えて、いくつかのライブラリ([implot](https://github.com/epezent/implot), [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo), and [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs)) もバンドルされており、迅速なプロトタイピングが可能です。
