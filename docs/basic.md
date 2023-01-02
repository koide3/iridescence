# Basic usage

## Creating a viewer instance

```guik::LightViewer::instance()``` creates and returns a global viewer instance. The viewer instance is created on only the first call, and results of ```guik::LightViewer::instance()``` refer to the same globally singular instance (i.e., singleton pattern).

```viewer->spin_once()``` updates viewer contents and renders a frame. It returns ```false``` when the viewer window is closed.

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

## Registering drawables to the viewer

3D objects to be drawn are called **drawables** and managed with unique names (or IDs) in Iridescence. The following code shows a minimum example to register a wire sphere to the viewer.

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  // Register a wire sphere drawable with the name "sphere"
  // and flat red coloring setting.
  viewer->update_drawable(
    "sphere",
    glk::Primitives::wire_sphere(),
    guik::FlatRed()
  );

  viewer->spin();
}
```

![Screenshot_20230102_233458](https://user-images.githubusercontent.com/31344317/210245441-2506d0c1-e044-4ed3-904d-00f9c96ef520.png)

The first argument of ```update_drawable()``` is a name to be assigned to the drawable. If the name already exists, the viewer overwrites the drawable with the new one.

The second argument is a drawable to be registered. Take a look at [Drawables](drawables.md) to see supported 3D drawable types in Iridescence.

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