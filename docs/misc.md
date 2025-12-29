# Miscellaneous

## Enabling/Disabling Vsync

By default, vsync is enabled and the maximum FPS is limited to the refresh rate of the display.
The maximum FPS can be unbounded by disabling vsync.

```cpp
// Disable vsync to unlimit the maximum FPS
viewer->disable_vsync();

// Enable vsync to limit the maximum FPS
viewer->enable_vsync();
```

## Spin methods

In addition to ```spin_once()``` and ```spin()```, there are two utility methods for spinning the viewer, ```spin_until_click()``` and ```toggle_spin_once()``` that are useful for debugging.

```spin_until_click()``` spins the viewer until the ```break``` button gets clicked for step-by-step debugging.
```cpp
double angle = 0.0;
while (viewer->spin_until_click()) {
  viewer->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow().rotate(angle, {0.0f, 0.0f, 1.0f}));
  angle += 0.1;
}
```

![until_click](https://user-images.githubusercontent.com/31344317/210242895-9bf043b1-1c30-4348-bd52-62ff4593da6d.gif)

```toggle_spin_once()``` spins the viewer and stops while the ```break``` checkbox is checked.
```cpp
double angle = 0.0;
while(viewer->toggle_spin_once()) {
  viewer->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow().rotate(angle, {0.0f, 0.0f, 1.0f}));
  angle += 0.01;
}
```
![toggle](https://user-images.githubusercontent.com/31344317/210242889-c0f5582c-b2dc-451e-8fb2-6cd1847460df.gif)

## Background color/image

```cpp
// Change the background color
viewer->set_clear_color({0.2f, 0.2f, 0.2f, 1.0f});

// Set a background image
std::shared_ptr<glk::Texture> texture = ...;
viewer->set_bg_texture(texture);
```

## Text output

```cpp
viewer->append_text("text1");
viewer->append_text("text2");
```

## Removing drawables

```cpp
// Remove a drawable with a specified name
viewer->remove_drawable("drawable_name");

// Removes drawables with names that match a regex pattern
viewer->remove_drawable(std::regex("drawable_.+"));

// Remove all drawables
viewer->clear_drawables();
```

## Drawable filter

```cpp
viewer->register_drawable_filter("filter", [](const std::string& drawable_name) {
  bool do_rendering = true;

  if (drawable_name == "drawable_to_be_filtered") {
    do_rendering = false;
  }

  return do_rendering;
});

// Drawable filter can be removed by overwriting with 0
viewer->register_drawable_filter("filter", 0);
```

## Changing the coloring settings of the Rainbow scheme

```cpp

glk::COLORMAP colormap = glk::COLORMAP::AUTUMN;   // Colormap type
Eigen::Vector2f range(-3.0f, 5.0f);               // Coloring range
Eigen::Vector3f axis(1.0f, 0.0f, 0.0f);           // Coloring axis

viewer->set_colormap(colormap);
viewer->shader_setting().add("z_range", range);
viewer->shader_setting().add("colormap_axis", axis);
```

## Colormaps

```cpp
#include <glk/colormap.hpp>

// Get colormap value (integer version: value range = [0, 255])
Eigen::Vector4i color = colormap(glk::COLORMAP::TURBO, 128);

// Get colormap value (float version: value range = [0.0, 1.0])
Eigen::Vector4f colorf = colormapf(glk::COLORMAP::TURBO, 0.5f);

// Get a caterogical color by evenly sampling colors from a colormap
// The color loops every "num_categories" counts
int count = 1;
int num_categories = 16;
Eigen::Vector4i cat_color = colormap_categorical(glk::COLORMAP::TURBO, count, num_categories);

// Float version
Eigen::Vector4f cat_colorf = colormap_categoricalf(glk::COLORMAP::TURBO, count, num_categories);
```

## Sub-viewer

```cpp
auto sub_viewer1 = viewer->sub_viewer("sub1");
sub_viewer1->update_drawable("cube", glk::Primitives::cube(), guik::Rainbow());

auto sub_viewer2 = viewer->sub_viewer("sub2");
sub_viewer2->update_drawable("sphere", glk::Primitives::sphere(), guik::Rainbow());
```

![Screenshot_20230102_222655](https://user-images.githubusercontent.com/31344317/210237883-97ec8b69-b0ec-4572-861d-2184aaa68485.png)


## Sharing the default camera control with sub-viewers

```cpp
auto camera_control = viewer->get_camera_control();
sub_viewer1->set_camera_control(camera_control);
sub_viewer2->set_camera_control(camera_control);
```

![subs](https://user-images.githubusercontent.com/31344317/210238057-629ea9ea-d439-4fa3-abcb-2d696edb7eee.gif)

## Taking screenshot

Simple but slow screen capture methods (1~30 FPS):
```cpp
// 8-bit RGBA pixel data
std::vector<unsigned char> color_pixels = viewer->read_color_buffer();
// float depth data
std::vector<float> depth_pixels = viewer->read_depth_buffer();
```

For efficient asynchronous screen data transfer with pixel buffer objects (~500FPS), see [src/example/ext_light_viewer_capture.cpp](https://github.com/koide3/iridescence/blob/master/src/example/ext_light_viewer_capture.cpp)

## File dialogs (portable-file-dialogs)

```cpp
#include <portable-file-dialogs.h>
#include <guik/recent_files.hpp>

guik::RecentFiles recent_files("input_directory");
const std::string path = pfd::select_folder("Select input directory", recent_files.most_recent()).result();
if (!path.empty()) {
  recent_files.push(path);
}
```

![Screenshot_20230103_003820](https://user-images.githubusercontent.com/31344317/210252758-45787529-1a65-4f67-8c73-030e467448a0.png)

## Logging (spdlog)

```cpp
#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>

#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>


// Setup a ringbuffer sink for the default spdlog logger
const int ringbuffer_size = 100;
auto ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(ringbuffer_size);

auto logger = spdlog::default_logger();
logger->sinks().emplace_back(ringbuffer_sink);
logger->set_level(spdlog::level::trace);

spdlog::trace("trace");
spdlog::debug("debug");
spdlog::info("info");
spdlog::warn("warning");
spdlog::error("error");


// Create a logger UI to display ringbuffer contents
const double bg_alpha = 0.7;
viewer->register_ui_callback("logging", guik::create_logger_ui(ringbuffer_sink, bg_alpha));
```

![Screenshot_20231212_111403](https://github.com/koide3/iridescence/assets/31344317/98ce4f8d-f009-44b8-9b37-490ec88e90f6)

## Image and 3D model IO

### PNG

```cpp
#include <glk/io/png_io.hpp>

// Load PNG image
// Pixel data are stored in 8-bit RGBA format
int width, height;
std::vector<unsigned char> pixels;
glk::load_png("image.png", width, height, pixels);

// Save image as a PNG image
// Pixel data must be 8-bit RGBA
glk::save_png("image.png", width, height, pixels);
```

### JPEG

```cpp
#include <glk/io/jpeg_io.hpp>

// Load JPEG image
// Pixel data are stored in 8-bit RGBA format
int width, height;
std::vector<unsigned char> pixels;
glk::load_jpeg("image.png", width, height, pixels);

// Save image as a JPEG image
// Pixel data must be 8-bit RGBA
int quality = 100;
glk::save_jpeg("image.png", width, height, pixels, quality);
```

### PLY

```cpp
#include <glk/io/ply_io.hpp>

// Load a PLY model
auto ply = glk::load_ply("model.ply");

// ply->vertices    : std::vector<Eigen::Vector3f>
// ply->colors      : std::vector<Eigen::Vector4f>
// ply->normals     : std::vector<Eigen::Vector3f>
// ply->intensities : std::vector<float>
// ply->indices     : std::vector<int>

// Save a PLY data
glk::save_ply_binary("model.ply", *ply);

// Save a point cloud in the PLY format
std::vector<Eigen::Vector3f> points
glk::save_ply_binary("model.ply", points.data(), points.size());
```

## Partial point cloud rendering

The rendering cost of large static point cloud can be mitigated with the partial rendering mode.
In this mode, only a part of static point clouds are rendered every frame and accumulated over time.
Although this causes flickering, it can drastically increase the rendering speed.

1. Enable the partial rendering mode for the viewer.  
    `viewer->enable_partial_rendering()`.
2. Create a point cloud buffer and set the points rendering budget.  
    ```cpp
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(...);
    int points_rendering_budget = 512;
    cloud_buffer->enable_partial_rendering(points_rendering_budget);
    ```
3. Mark the drawable as a static object.
    ```cpp
    viewer->update_drawable("points", cloud_buffer, guik::Rainbow().static_object());
    ```
4. Mark other objects as dynamic.
    ```cpp
    viewer->update_cube("cube", guik::FlatBlue().dynamic_object());
    ```

See also [ext_light_viewer_partial_rendering.cpp](https://github.com/koide3/iridescence/blob/master/src/example/ext_light_viewer_partial_rendering.cpp).

![partial_rendering](https://github.com/koide3/iridescence/assets/31344317/1a54035f-3faf-4910-8a3a-97d0cfb71a1e)

!!!note
    This feature may not work well on non-NVIDIA GPUs.

## Viewer menu

By pressing "Ctrl+M", a hidden menu bar appears. Via the manu bar, you can:

- Change the rainbow colormap, coloring axis and range
- Show an information window (FPS/CPU&GPU Usage)
- Enable/Disable vsync
- Enable/Disable XY grid
- Show drawable filter and editor
- Save/Load the camera setting
- Get 3D positions of objects via point picking

![Screenshot_20230102_202624](https://user-images.githubusercontent.com/31344317/210225203-e6edf5d8-d495-413b-b554-15fb61294923.png)


## Keyboard shortcut

### General

| Key                   | Description                     |
| --------------------- | ------------------------------- |
| Ctrl + M              | Show viewer menu                |
| Ctrl + J              | Save screenshot                 |
| Ctrl + F              | Fit all plots to data           |
| Ctrl + MINUX / PLUS   | Increase / decrease point scale |

### Camera control

| Key                    | Description                                    |
| ---------------------- | ---------------------------------------------- |
| Ctrl + Arrow           | Move camera                                    |
| Ctrl + Page UP / Down  | Zoom in/out                                    |
| Ctrl + Home / End      | Increase / decrease moving speed (permanently) |
| Shift                  | Increase moving speed (while holding)          |

