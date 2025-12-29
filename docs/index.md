![Iridescence](assets/logo.png)

**Iridescence** a light-weight visualization library for rapid prototyping of 3D algorithms. This library is designed for accelerating personal research and development projects (mainly focusing on point-cloud-related algorithms) and is NOT intended to be a general-purpose visualization library with rich rendering capabilities.

[![Build](https://github.com/koide3/iridescence/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/iridescence/actions/workflows/build.yml) on Ubuntu 18.04 / 20.04 / 22.04

## Features

What this library provides:

- An easy-to-use 3D visualization framework (inpaticular suitable for rendering point clouds)
- Tightly integrated Dear ImGui interfaces for rapid UI design

What this library does NOT provide:

- Realistic rendering and shading
- Rich textured 3D mesh rendering

## Dependencies

- [GLFW](https://www.glfw.org/) ([zlib/libpng license](https://www.glfw.org/license.html))
- [gl3w](https://github.com/skaslev/gl3w) ([Public domain](https://github.com/skaslev/gl3w/blob/master/UNLICENSE))
- [Dear ImGui](https://github.com/ocornut/imgui) ([MIT license](https://github.com/ocornut/imgui/blob/master/LICENSE.txt))
- [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo) ([MIT license](https://github.com/CedricGuillemet/ImGuizmo/blob/master/LICENSE))
- [implot](https://github.com/epezent/implot) ([MIT license](https://github.com/epezent/implot/blob/master/LICENSE))
- [Eigen](https://eigen.tuxfamily.org/index.php) ([MPL2 license](https://www.mozilla.org/en-US/MPL/2.0/))
- [portable-file-dialogs](https://github.com/samhocevar/portable-file-dialogs) ([WTFPL license](https://github.com/samhocevar/portable-file-dialogs/blob/main/COPYING))

## Installation

### Install from source

```bash
# Install dependencies
sudo apt-get install -y libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev

# Build and install Iridescence
git clone https://github.com/koide3/iridescence --recursive
mkdir iridescence/build && cd iridescence/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# [Optional] Build and install python bindings
cd ..
pip install .

# [Optional2] Install stubs for autocomplete
pip install pybind11-stubgen
cd ~/.local/lib/python3.10/site-packages
pybind11-stubgen -o . --ignore-invalid=all pyridescence
```

### Install from [PPA](https://github.com/koide3/ppa) [AMD64, ARM64]

#### Ubuntu 24.04

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2404/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2404 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list

sudo apt update && sudo apt install -y libiridescence-dev
```

#### Ubuntu 22.04

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2204/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list

sudo apt update && sudo apt install -y libiridescence-dev
```

#### Ubuntu 20.04

```bash
curl -s --compressed "https://koide3.github.io/ppa/ubuntu2004/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null
echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2004 ./" | sudo tee /etc/apt/sources.list.d/koide3_ppa.list

sudo apt update && sudo apt install -y libiridescence-dev
```

## Use Iridescence in your cmake project

```cmake
# Find package
find_package(Iridescence REQUIRED)


# Add include dirs and link libraries
add_executable(your_program
  src/your_program.cpp
)
target_link_libraries(your_program
  Iridescence::Iridescence
)
```


## Minimum example

C++:
```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  // Create a viewer instance (global singleton)
  auto viewer = guik::LightViewer::instance();

  float angle = 0.0f;

  // Register a callback for UI rendering
  viewer->register_ui_callback("ui", [&]() {
    // In the callback, you can call ImGui commands to create your UI.
    // Here, we use "DragFloat" and "Button" to create a simple UI.
    ImGui::DragFloat("Angle", &angle, 0.01f);

    if (ImGui::Button("Close")) {
      viewer->close();
    }
  });

  // Spin the viewer until it gets closed
  while (viewer->spin_once()) {
    // Objects to be rendered are called "drawables" and managed with unique names.
    // Here, solid and wire spheres are registered to the viewer respectively with the "Rainbow" and "FlatColor" coloring schemes.
    // The "Rainbow" coloring scheme encodes the height of each fragment using the turbo colormap by default.
    Eigen::AngleAxisf transform(angle, Eigen::Vector3f::UnitZ());
    viewer->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow(transform));
    viewer->update_drawable("wire_sphere", glk::Primitives::wire_sphere(), guik::FlatColor({0.1f, 0.7f, 1.0f, 1.0f}, transform));
  }

  return 0;
}
```

<details>
  <summary>Python version</summary>

```py
#!/usr/bin/python3
import numpy
from scipy.spatial.transform import Rotation
from pyridescence import *

# Create a viewer instance (global singleton)
viewer = guik.LightViewer.instance()

angle = 0.0

# Define a callback for UI rendering
def ui_callback():
  # In the callback, you can call ImGui commands to create your UI.
  # Here, we use "DragFloat" and "Button" to create a simple UI.

  global angle
  _, angle = imgui.drag_float('angle', angle, 0.01)

  if imgui.button('close'):
    viewer.close()

# Register a callback for UI rendering
viewer.register_ui_callback('ui', ui_callback)

# Spin the viewer until it gets closed
while viewer.spin_once():
  # Objects to be rendered are called "drawables" and managed with unique names.
  # Here, solid and wire spheres are registered to the viewer respectively with the "Rainbow" and "FlatColor" coloring schemes.
  # The "Rainbow" coloring scheme encodes the height of each fragment using the turbo colormap by default.
  transform = numpy.identity(4)
  transform[:3, :3] = Rotation.from_rotvec([0.0, 0.0, angle]).as_matrix()
  viewer.update_drawable('sphere', glk.primitives.sphere(), guik.Rainbow(transform))
  viewer.update_drawable('wire_sphere', glk.primitives.wire_sphere(), guik.FlatColor(0.1, 0.7, 1.0, 1.0, transform))

```
</details>


![example_01](https://user-images.githubusercontent.com/31344317/210127177-31630466-f8a1-45b6-8bc7-2fdd2e4c9548.gif)


## Some use examples in my academic works

![ral2021](https://user-images.githubusercontent.com/31344317/210128637-80f79abf-69c3-479c-91e9-0807e5b8b3ae.jpg)
![iros2022](https://user-images.githubusercontent.com/31344317/210128635-2ef02dff-3d74-499e-bde8-2c9c0dc047ff.jpg)


## License

This package is released under the MIT license.
