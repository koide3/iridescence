# Basic usage

## Creating a viewer instance

```guik::LightViewer::instance()``` creates and returns a global viewer instance. The viewer instance is created on the first call, and results of ```guik::LightViewer::instance()``` refer to the same globally singular instance (i.e., singleton pattern).

```viewer->spin_once()``` updates viewer contents and renders a frame on the window. It returns ```false``` when the viewer window is closed.

The following code is a minimum example to show a blank viewer window.

```cpp
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  // Create a global viewer instance
  auto viewer = guik::LightViewer::instance();

  // Spin the viewer until the window gets closed
  while (viewer->spin_once()) {}
}
```

![Screenshot_20230102_223722](https://user-images.githubusercontent.com/31344317/210238748-b4f463ee-af08-425c-86d3-7b21ae4b18e5.png)

Instead of calling ```spin_once()``` in a while loop, you can also use ```spin()``` that spins the viewer until the windows gets closed.

```cpp
viewer->spin();
// The above is equivalent to the below
while(viewer->spin_once()) {}
```

There is also a shorthand function ```guik::viewer()``` that is equivalent to ```guik::LightViewer::instance()```.

```cpp
auto viewer =  guik::viewer();  // Equivalent to guik::LightViewer::instance()
```


With these convenient functions, the minimum example can be rewritten in a shorter form.

```cpp
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();
  viewer->spin();
}
```

## Registering drawables to the viewer

3D objects to be drawn are called **drawables** and managed with unique names (or IDs). The following code shows a minimum example to register a wire sphere to the viewer.

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  // Register a wire sphere drawable with the name "sphere"
  // and the flat red coloring setting.
  viewer->update_drawable(
    "sphere",
    glk::Primitives::wire_sphere(),
    guik::FlatRed()
  );

  viewer->spin();
}
```

![Screenshot_20230102_233458](https://user-images.githubusercontent.com/31344317/210245441-2506d0c1-e044-4ed3-904d-00f9c96ef520.png)

The first argument of ```update_drawable()``` is a name to be assigned to the drawable. If the name already exists, the viewer overwrites the existing drawable with the new one.

The second argument is a drawable to be registered. Take a look at [Drawables](drawables.md) to see supported 3D drawable types.

The third argument is a shader setting to hold rendering parameters. By changing shader setting parameters, you can control the color, pose, and shape of drawables as shown in below:
```cpp
auto transformation1 = Eigen::Translation3f(0.0f, -2.0f, 0.0f);
auto setting1 = guik::Rainbow(transformation1);
viewer->update_drawable("sphere1", glk::Primitives::wire_sphere(), setting1);

auto transformation2 = Eigen::Translation3f(0.0f, 2.0f, 0.0f);
auto setting2 = guik::FlatColor({1.0f, 0.5f, 0.2f, 1.0f}, transformation2);
viewer->update_drawable("sphere2", glk::Primitives::wire_sphere(), setting2);
```

![Screenshot_20230102_235102](https://user-images.githubusercontent.com/31344317/210247287-5f51f5c7-58a2-4c55-bed0-b1245bad6bde.png)

See [Shader setting](shader.md) for more details.


!!!tip
    There are shorthand functions (e.g., ```update_sphere```) for frequently used drawable types that allow registering drawables without including additional headers.

    ```cpp
    // Register a solid sphere
    viewer->update_sphere("sphere", guik::FlatRed());
    ```

!!!tip
    ShaderSetting class has several utility methods for handy manipulation of the model matrix.

    ```cpp
    // Register a solid sphere with translation and scaling
    viewer->update_sphere("sphere", guik::FlatRed().translate({1.0, 2.0, 3.0}).scale(0.1));
    ``` 


## Registering UI callbacks (Dear ImGui)

Research and development often involve trial-and-error processes with many parameter settings that often take a large amount of effort. To accelerate such development processes, Iridescence provides tightly integrated interfaces to [Dear ImGui](https://github.com/ocornut/imgui), an immediate mode GUI library that enables designing interactive user interfaces easily and rapidly.

To create a ImGui-based GUI, register a callback function for UI rendering events using ```register_ui_callback()``` . In the following example, we create a simple GUI with ```DragFloat``` to rotate a sphere, and ```Button``` to close the viewer window.

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  float angle = 0.0f;

  // Register a callback for UI rendering with the name "ui_callback".
  viewer->register_ui_callback("ui_callback", [&]() {
    // In the callback, you can call ImGui commands to create your UI.

    ImGui::DragFloat("Angle", &angle, 0.01f);

    if (ImGui::Button("Close")) {
      viewer->close();
    }
  });

  while (viewer->spin_once()) {
    // Show rotated solid and wire spheres.
    viewer->update_drawable("sphere", glk::Primitives::sphere(),
      guik::Rainbow().rotate(angle, {0.0f, 0.0f, 1.0f}));
    viewer->update_drawable("wire_sphere", glk::Primitives::wire_sphere(),
      guik::FlatColor(0.1f, 0.7f, 1.0f, 1.0f).rotate(angle, {0.0f, 0.0f, 1.0f}));
  }

  return 0;
}
```

![example_01](https://user-images.githubusercontent.com/31344317/210127177-31630466-f8a1-45b6-8bc7-2fdd2e4c9548.gif)

See [Dear ImGui](https://github.com/ocornut/imgui) for details of the GUI library.

!!!note 
    In addition to Dear ImGui, several libraries ([implot](https://github.com/epezent/implot), [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo), and [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs)) are bundled for rapid prototyping.
