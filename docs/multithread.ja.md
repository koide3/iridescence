# マルチスレッド

## Invoke

OpenGLコマンドは単一のGUIコンテキストスレッドでのみ実行できるため、Iridescenceの機能のほとんども単一のGUIスレッドで実行する必要があります。描画コンテンツを別スレッドから安全に更新するため、タスク（ファンクタ）をスレッドセーフなタスクキューにプッシュし、GUIスレッドで実行するための```invoke()```メソッドが用意されています。

```cpp

// この関数はバックグラウンドスレッドで実行されます
void async_update() {
  for (int i=0; i<100; i++) {
    // レンダリングする点群データを準備
    std::shared_ptr<std::vector<Eigen::Vector3f>> points = ...;

    // OpenGLオブジェクトはOpenGLコンテキストが作成されたスレッドで作成する必要があるため、
    // ここでPointCloudBufferを作成することはできません（クラッシュする）。
    // 
    // 次の行はこの関数内では実行できない!!
    // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(*points);

    auto viewer = guik::LightViewer::instance();
    // ビューワに次の関数をGUIスレッドで実行するように要求します。
    viewer->invoke([=] {
      // この関数は後でGUIスレッドで実行されるため、ここでOpenGLオブジェクトを作成できます。
      auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(*points);
      viewer->update_drawable("points", cloud_buffer, guik::Rainbow());
    });
  }
}

int main(int argc, char** argv) {
  // OpenGLコンテキストは、LightViewer::instance() の最初の呼び出しが行われたスレッドで作成されます。
  // したがって、基本的には最初にメインスレッドで呼び出すことをお勧めします。
  auto viewer = guik::LightViewer::instance();

  std::thread thread(async_update);

  viewer->spin();

  return 0;
}
```

## スレッドセーフな操作

```append_text``` と ```clear_text``` はスレッドセーフです。デバッグやロギング用のテキストをビューワに表示することができます。

```cpp
viewer->append_text("test");
viewer->clear_text();
```

## AsyncViewer

プログラムが何らかの重たい処理を行っている間もビューワをインタラクティブに保ちたい場合は、`guik::AsyncLightViewer`が利用できませす。このクラスはバックグラウンドスレッドでビューワインスタンスを作成し、スレッドセーフにオブジェクト生成・登録が可能です。`06_light_viewer_async.cpp` を参照してください。

```cpp
#include <guik/viewer/async_light_viewer.hpp>

int main(int argc, char** argv) {
  // AsyncViewerはバックグラウンドスレッドでビューワを作成して実行します。
  auto async_viewer = guik::async_viewer();

  // 安全なデータ操作のためにAsyncViewerのメソッドを使用してください。
  // AsyncViewerはメインスレッドがスリープしている間もインタラクティブに保たれます。
  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere1", guik::FlatRed().translate(0.0f, 0.0f, 0.0f));

  std::this_thread::sleep_for(std::chrono::seconds(1));
  async_viewer->update_wire_sphere("sphere2", guik::FlatGreen().translate(2.0f, 0.0f, 0.0f));

  // ビューワが閉じられるのを待ちます。
  guik::async_wait();
}
```

!!! warning
    AsyncViewerはバックグラウンドスレッドでビューワを実行するため、メインスレッドで標準のビューワ関数を呼び出すことはできません。

    ```cpp
      auto async_viewer = guik::async_viewer();

      // 下の行はプログラムのクラッシュや不正な動作を引き起こす可能性があります。
      // guik::viewer()->update_sphere("sphere0", guik::FlatBlue());
    ```
