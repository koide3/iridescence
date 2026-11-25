# Iridescence: guide for coding agents

Iridescence is an OpenGL/Dear ImGui viewer library for rapid prototyping of 3D algorithms (point clouds, trajectories, maps, plots, images). It has a C++ API and a Python binding (`pyridescence`) that mirrors it. This file describes how to write visualization code with either one.

| Path | Content |
|---|---|
| `include/guik/` | Viewer, shader settings, cameras, UI helpers. `viewer/light_viewer.hpp` and `viewer/light_viewer_context.hpp` list every viewer method. |
| `include/glk/` | Drawables, textures, colormaps, IO. |
| `src/example/` | Runnable examples, `NN_*.cpp` with a `NN_*.py` counterpart. **Start from the closest one.** |
| `docs/*.md` | Topic guides (basic, drawables, shader, picking, multithread, effects, cookbook). |
| `src/python/pyridescence/` | Python package: type stubs (`*.pyi`) and [`AGENTS.md`](src/python/pyridescence/AGENTS.md), the Python-only version of this guide that is installed with the wheel. |
| `src/python/*.cpp` | pybind11 bindings. A function missing here is not available in Python. |

Take signatures from the headers (C++) or the stubs (Python) rather than from memory: both APIs have many overloads, and the Python binding covers a subset.

## Mental model (both languages)

- There is **one viewer window per process**: `guik::viewer()` (same as `guik::LightViewer::instance()`) creates it on the first call and returns the same object afterwards. The arguments (`size`, `background`, `title`) are used only by the first call.
- Things to draw are **drawables registered under unique string names**. `update_*(name, ...)` creates the drawable or replaces the one with the same name; `remove_drawable(name)` deletes it. There is no scene graph.
- Every drawable comes with a **`ShaderSetting`** that holds its color scheme, model matrix (pose), and point size.
- The UI is immediate-mode: a **UI callback** registered with `register_ui_callback(name, fn)` runs every frame and calls Dear ImGui.
- Nothing is drawn until the viewer **spins**: `spin_once()` renders one frame and returns `false` once the window is closed; `spin()` blocks until then.
- OpenGL is single-threaded: the viewer and every drawable must be created and used on one thread (normally the main thread). Violations crash instead of reporting an error.

## C++

### Build and minimal program

```cmake
find_package(Iridescence REQUIRED)
target_link_libraries(your_program Iridescence::Iridescence)   # C++17
```

```cpp
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

int main() {
  auto viewer = guik::viewer();
  std::vector<Eigen::Vector3f> points = /* ... */;
  float point_size = 0.05f;

  viewer->register_ui_callback("ui", [&] {
    ImGui::Begin("control", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::DragFloat("point size [m]", &point_size, 0.001f, 0.001f, 1.0f)) {
      viewer->set_point_shape(point_size, true, true);  // (size, metric, circle) for all points
    }
    if (ImGui::Button("close")) {
      viewer->close();
    }
    ImGui::End();
  });

  viewer->set_point_shape(point_size, true, true);
  viewer->update_points("points", points, guik::Rainbow());               // color = height (Z)
  viewer->update_coord("origin", guik::VertexColor().scale(0.5f));        // XYZ axes in RGB
  viewer->update_thin_lines("path", points, true, guik::FlatGreen());     // true = line strip

  while (viewer->spin_once()) {
    // update drawables here
  }
  return 0;
}
```

`light_viewer.hpp` pulls in ImGui, Eigen, and the shader settings. Include the header of each `glk` class you construct yourself (`glk/pointcloud_buffer.hpp`, `glk/thin_lines.hpp`, ...) and of each camera class whose methods you call (`guik/camera/sensor_view_camera_control.hpp`, ...); the viewer headers only forward-declare them.

### Run loop and threads

| Situation | Use |
|---|---|
| Static scene, or everything happens in UI callbacks | `viewer->spin()` |
| A processing loop that also draws (the common case) | `while (viewer->spin_once()) { step(); viewer->update_...(); }` |
| Step-by-step debugging of an algorithm | `spin_until_click()` (blocks until a "break" button is clicked), `toggle_spin_once()` |
| Algorithm on a worker thread, viewer on the main thread | `viewer->invoke(fn)` from the worker |
| Main thread must not be blocked by the viewer | `guik::async_viewer()` (`guik/viewer/async_light_viewer.hpp`) |
| Viewer owned by a component (plugin, ROS node) | A dedicated thread that creates the viewer, loops `spin_once()`, and calls `guik::destroy()` before exiting |

- `viewer->invoke([=] { ... })` queues a function that runs on the viewer thread before the next frame. Capture the data **by value or `shared_ptr`**, never by reference to worker-thread locals. `invoke_once(label, fn)` keeps only the latest request per label; `invoke_after_rendering` runs after the frame.
- Only `append_text()` / `clear_text()` may be called directly from other threads. Everything else, including constructing `glk::PointCloudBuffer`, `glk::ThinLines`, or `glk::Texture`, goes through `invoke`.
- When the viewer runs in its own thread, that thread must create it (first `guik::viewer()` call), do all GL work, and destroy it. Shared state touched by UI callbacks needs a mutex.
- `guik::async_viewer()` runs the viewer in a background thread and exposes thread-safe `update_*`, camera, plot, and image methods that copy the data. Do not call `guik::viewer()` from the main thread while it is alive (inside its UI callbacks it is fine). To pass a custom drawable, wrap its construction: `std::make_shared<glk::DelayedConstructor>([data] { return std::make_shared<glk::ThinLines>(data, true); })` (`glk/delayed_constructor.hpp`). End with `guik::async_wait()` or `guik::async_destroy()`.

### Drawables

Shorthands on the viewer (and on sub-viewers):

| Call | Notes |
|---|---|
| `update_points(name, points, setting)` | `std::vector<Eigen::Vector3f/4f/3d/4d>`, or `(const float* data, int stride_bytes, int num_points)`. Returns `std::shared_ptr<glk::PointCloudBuffer>`. |
| `update_thin_lines(name, vertices, [colors], [indices], line_strip, [line_width], setting)` | `line_strip = true`: polyline through all vertices (trajectories). `false`: independent segments from vertex pairs. Width in pixels. |
| `update_coord(name, setting)` | Coordinate frame; needs `guik::VertexColor(pose)` to show RGB axes. |
| `update_sphere / cube / cone / icosahedron / frustum`, `update_wire_*` | Unit primitives placed and sized by the model matrix of the setting. |
| `update_normal_dists(name, means, covs, sigma, setting)` | Ellipsoids of Gaussians. |
| `update_drawable(name, drawable, setting)` | Any `glk::Drawable` as a `shared_ptr`. |

`glk` drawables for `update_drawable` (construct with `std::make_shared`):

- `glk::Primitives::sphere()`, `cube()`, `cone()`, `icosahedron()`, `bunny()`, `frustum()`, `coordinate_system()` and `wire_*()`: cached singletons, cheap to pass every frame.
- `glk::PointCloudBuffer(points)` with per-point attributes: `add_color(std::vector<Eigen::Vector4f>)`, `add_intensity(glk::COLORMAP::TURBO, std::vector<float>)`, `add_colormap(std::vector<float>)`, `add_normals(...)`, `add_buffer(attribute_name, data)`. `glk::IndexedPointCloudBuffer(buffer, indices)` draws a subset without copying. PCL clouds: `glk::create_point_cloud_buffer(cloud)` in `glk/pointcloud_buffer_pcl.hpp`.
- `glk::ThinLines(vertices, [colors], line_strip, line_width)`, `glk::Lines(width_m, vertices, [colors])` (thick lines, width in meters), `glk::Trajectory`, `glk::PointNormalsBuffer`, `glk::PointCorrespondences`.
- `glk::Mesh` (raw pointers with strides, plus indices), `glk::NormalDistributions`.
- Maps: `glk::VoxelMap(voxel_coords, resolution)` (voxel `c` covers `[c, c + 1] * resolution`), `glk::GridMap(resolution, width, height, values, alpha, mode)` (2D textured grid, draw with `guik::TextureColor`), `glk::ElevationMap(glk::ElevationMapBuilder)`; see `src/example/02_light_viewer_maps.cpp`.
- `glk::DrawableContainer{drawables...}` groups drawables under one name.
- Custom rendering: subclass `glk::Drawable` and implement `draw(glk::GLSLShader&) const` (`src/example/ext_light_viewer_custom.cpp`).

Naming and lifetime:

- Name drawables with a group prefix and an ID: `"frame_" + std::to_string(i)`, `"submap_12"`, `"map_occupancy"`. Prefixes make group operations one-liners:
  - `viewer->remove_drawable(std::regex("frame_.*"))`: the regex must match the **whole** name (`std::regex_match`).
  - `viewer->register_drawable_filter("f", [&](const std::string& name) { return show_frames || name.rfind("frame_", 0) != 0; })`: hide a group without deleting it. This is the right tool for "show X" checkboxes over data that is expensive to rebuild.
- `auto found = viewer->find_drawable(name)` returns `{ShaderSetting::Ptr, Drawable::ConstPtr}`, both null if missing. Mutating `found.first` (`set_model_matrix`, `set_color`, `set_alpha`, `add`, ...) takes effect immediately without re-uploading geometry. Use it to move or recolor existing objects, e.g. after poses were re-optimized.
- `remove_drawable` on a missing name is a no-op. `clear_drawables()` removes everything. `guik::anon()` returns a fresh unique name.

### ShaderSetting: color, pose, point size

| Setting | Color comes from |
|---|---|
| `guik::Rainbow()` | Position along Z (default range -3 to 5 m), through the viewer colormap. |
| `guik::FlatColor(r, g, b, a)`, `FlatColor({r, g, b, a}, T)`, `FlatRed/Green/Blue/Orange/White/Gray/DarkGray/Black()` | One color, components in [0, 1]. |
| `guik::VertexColor()` | Per-vertex colors of the drawable (`add_color`, `add_intensity`, colors of lines and meshes, axes of `update_coord`). |
| `guik::TextureColor()` | Texture of the drawable (`glk::GridMap`, textured meshes). |
| `guik::VertexColorMap()` | Per-vertex scalar (`add_colormap`) mapped on the GPU with the viewer colormap over `set_colormap_range`. |

- A setting whose mode has no matching data renders black or wrong: `VertexColor` on a cloud without colors, or `FlatColor` on `update_coord` (gives single-colored axes).
- Ranges: `.set_rainbow_range(Eigen::Vector2f(zmin, zmax))` for `Rainbow` and `.set_colormap_range(...)` for `VertexColorMap`, per drawable or for everything via `viewer->shader_setting()`. Geometry outside the Z range saturates to one color, so match the range to the data. `viewer->set_colormap(glk::COLORMAP::TURBO)` selects the colormap; `glk::colormapf(cmap, 0..1)` and `glk::colormap_categoricalf(cmap, i, n)` give colors for your own use (`glk/colormap.hpp`).
- Alpha < 0.999 in a flat color (or `set_alpha`) switches the drawable to the transparent pass. For settings without a flat color, call `.make_transparent()`.
- **`add(name, value)` deduces the uniform type from the exact C++ type**: `int`, `float`, `Eigen::Vector2f/3f/4f`, `Eigen::Vector2i/3i/4i`, `Eigen::Matrix4f`. A `double` literal (`add("point_scale", 2.0)`) or `Vector3d` produces an invalid parameter and a "type mismatch" error; write `2.0f` and cast Eigen types to float. Prefer the typed setters when one exists.
- Setters return `ShaderSetting&`, so calls chain on a temporary.

Pose: every setting constructor takes an optional transform as the last argument, of any Eigen type (`Matrix4f/d`, `Isometry3f/d`, `Affine3f/d`); `double` poses are cast for you.

```cpp
guik::VertexColor(T_world_body).scale(0.3f)                 // pose from a transform, then shrink the axes
guik::FlatRed().translate(pt).scale(0.1f)                   // small sphere/cube at a point
guik::FlatColor(0.2f, 0.9f, 0.4f, 0.35f, T_box * Eigen::Scaling(sx, sy, sz))   // box of a given size
```

- `translate()` adds a world-frame offset and `scale()` scales the object about its own origin, in any order. `rotate()` and `transform()` right-multiply the current matrix.
- Keep sensor-frame data in its own frame and put the pose in the model matrix (`update_points("scan", scan, guik::FlatOrange(T_world_sensor))`) instead of transforming the points. Moving an object is then a matter of replacing the setting.
- The world is right-handed and Z-up by default (the grid is the XY plane). For camera-style frames use `use_orbit_camera_control_xz()`.

Point size: `viewer->set_point_shape(size, metric, circle)` sets the default for all clouds; with `metric = true`, `size` is a radius in meters (`0.02` suits indoor scans, larger for outdoor maps), otherwise pixels. Per drawable, `.set_point_scale(1.5f)` multiplies the default (highlight the current scan over the map) and `.set_point_size(0.05f)` / `.set_point_shape(...)` override it.

### UI

- UI callbacks call plain Dear ImGui (`ImGui::Begin`, `ImGui::DragFloat`, ...), ImPlot, and ImGuizmo; the headers are installed with the library. `ImGui::ShowDemoWindow()` shows every widget.
- Every `ImGui::Begin()` needs `ImGui::End()`, also on early returns. `BeginMenu`, `BeginPopup`, `BeginTable`, `BeginCombo` need their `End*` only when they returned `true`.
- Callbacks are stored and run every frame: lambdas must not capture references to locals that die before the viewer does. Capture `this` or a `shared_ptr`.
- Do cheap viewer updates directly in the callback (it runs on the viewer thread). Run heavy work on a thread and bring the result back with `invoke`; `guik::ProgressModal` (`guik/progress_modal.hpp`) packages this as `open<T>(name, task)` when a button is clicked plus `run<T>(name)` every frame, which draws a progress popup and returns `std::optional<T>` once the task finished.
- Helpers: `viewer->append_text(str)` (log window), `guik::ModelControl` (pose gizmo, `model_matrix()`), `guik::HoveredDrawings` (text and markers pinned to 3D points or drawables; register `create_callback()`), `guik::RecentFiles(tag)` (remembers paths across runs), `guik::create_logger_ui` (spdlog sink window, `guik/spdlog_sink.hpp`), and `pfd::open_file / save_file / select_folder(...).result()` (native dialogs, `portable-file-dialogs.h`; an empty result means cancel).
- `viewer->sub_viewer(name, {w, h})` opens a second 3D view in its own window with the same drawable and camera API, for showing a different scene (a local map, a selected item) next to the main one.

### Camera

- `use_orbit_camera_control(distance, theta, phi)` (default; Z-up, radians), `use_orbit_camera_control_xz`, `use_topdown_camera_control(distance, theta)`, `use_arcball_camera_control`, `use_fps_camera_control(fovy_deg)`, `use_sensor_view_camera_control(T_sensor_camera)` (chase camera; call `set_sensor_pose(T_world_sensor)` on the returned object every frame).
- `viewer->lookat(pt)` moves the camera center; call it every frame behind a "follow" checkbox to track a moving sensor. `reset_center()` returns to the origin.
- Set `distance` to the scale of the scene. The far plane is 1000 by default: `use_perspective_projection_control(fovy_deg, near, far)`, `use_orthographic_projection_control(width)`.
- `disable_xy_grid()` hides the ground grid; `set_clear_color(Eigen::Vector4f)` sets the background; `save_camera_settings(path)` / `load_camera_settings(path)` persist the view.

### Plots and images

```cpp
viewer->setup_plot("position [m]", 520, 170);              // optional: size and ImPlot flags
viewer->update_plot_line("position [m]", "x", ts, xs);     // (plot, label, xs, ys) or (plot, label, ys)
viewer->fit_plot("position [m]");                          // refit the axes to the data
viewer->update_image("camera", texture, 0.5);              // std::shared_ptr<glk::Texture>
```

- A plot/label or image name identifies the item; updating with the same names replaces the data. Vectors of any arithmetic type are accepted, as are `std::vector<Eigen::Vector2d>` and a vector of structs with a lambda that maps each element to `(x, y)`. The whole series is passed every time, so cap the history for real-time plots.
- Axes do not follow new data on their own: call `fit_plot` / `fit_all_plots`, or pass `ImPlotAxisFlags_AutoFit` to `setup_plot`.
- A name of the form `"group/name"` puts plots (and images) with the same group in one tabbed window.
- Also `update_plot_scatter`, `update_plot_stairs`, `update_plot_histogram`, `link_plot_axes`, `set_line_style`, `remove_plot` (`src/example/07_light_viewer_plots.cpp`).
- Textures: `glk::create_texture(cv::Mat)` (`glk/texture_opencv.hpp`, needs OpenCV in your project), or `std::make_shared<glk::Texture>(Eigen::Vector2i(w, h), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, pixels)`.

### Picking and capture

- In a UI callback: `if (auto pt = viewer->pick_point(ImGuiMouseButton_Right)) { ... }` yields the 3D point under a click (`std::optional<Eigen::Vector3f>`). Lower level: `pick_depth(pixel)` (>= 1.0 is the background) and `unproject(pixel, depth)`; guard manual mouse handling with `if (ImGui::GetIO().WantCaptureMouse) return;`.
- To identify the clicked object, call `viewer->enable_info_buffer()`, tag settings with `.add("info_values", Eigen::Vector4i(id, 0, 0, 0))`, and read `viewer->pick_info(pixel)` (all `-1` on the background). See `src/example/04_light_viewer_picking.cpp`.
- `save_color_buffer("shot.png")`, `read_color_buffer()` (RGBA bytes, top row first), and `read_depth_buffer()` return the frame of the last `spin_once()`, so spin after the last update and before reading.
- Offscreen rendering: `guik::viewer(Eigen::Vector2i(1280, 720), true)` creates a hidden window; then loop over update, `spin_once()`, read. It still needs an OpenGL context, i.e. a display (`xvfb-run` on headless machines).

### Performance and pacing

- `spin_once()` waits for vsync and can block entirely while the window is hidden or the screen is locked. For processing loops call `viewer->disable_vsync()` once and limit the frame rate yourself if needed.
- Do not spin once per processed item when processing is faster than the display; spin when 1/30 to 1/60 s has passed since the last frame.
- Each `update_points` uploads a buffer. Re-upload large, slowly changing clouds (accumulated maps) every N frames or a few times per second; update small things (current scan, pose, trajectory) every frame. When only the pose or color changes, mutate the setting from `find_drawable` and keep the buffer.
- For scenes with many millions of points: `viewer->enable_partial_rendering()` redraws only drawables marked `.dynamic_object()` on top of a cached static frame (`src/example/ext_light_viewer_partial_rendering.cpp`); `glk::IndexedPointCloudBuffer` or downsampling reduce what is drawn.
- Screen-space effects: `viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>())` adds depth cues to dense clouds; `glk::ScreenSpaceLighting` additionally needs `viewer->enable_normal_buffer()` and per-point normals (`src/example/ext_light_viewer_effects.cpp`).

## Python (pyridescence)

`pip install pyridescence`, then `from pyridescence import glk, guik, imgui, pfd, gl`. Everything above applies with the same names; the full Python guide is [`src/python/pyridescence/AGENTS.md`](src/python/pyridescence/AGENTS.md) (installed as `AGENTS.md` in the package directory, next to `examples/` and the `.pyi` stubs). The differences that matter:

| C++ | Python |
|---|---|
| `guik::viewer()->update_points(...)` | `guik.viewer().update_points(...)`; data are numpy arrays, (N, 3) or (N, 4), any dtype (converted to `float32`). |
| `PointCloudBuffer::add_color / add_intensity / add_colormap` | Keyword arguments: `viewer.update_points(name, points, colors=, intensities=, cmaps=, normals=, shader_setting=)`. |
| `update_thin_lines(name, pts, true, setting)` | `update_thin_lines(name, pts, line_strip=True, shader_setting=setting)`; the setting **must be a keyword**. |
| Transforms of any Eigen type | 4x4 numpy matrices; also `guik.Rainbow(scale=, trans=, rot=)`. |
| `guik::FlatColor(r, g, b)` | All four components are required: `guik.FlatColor(r, g, b, a)`. |
| `add(name, value)` with any uniform type; `set_rainbow_range`, `set_colormap_range` | `add()` takes a float, 2-vector, or 4x4 matrix; `addi()` an int 4-vector. Ranges: `.add('z_range', np.float32([lo, hi]))`, `.add('cmap_range', ...)`. |
| `ImGui::DragFloat("x", &v)` returns `bool`, writes through the pointer | `changed, v = imgui.drag_float('x', v)`; snake_case names, constants without the `ImGui` prefix (`imgui.WindowFlags_AlwaysAutoResize`). |
| `remove_drawable(std::regex("frame_.*"))` | `remove_drawable('frame_.*', True)`. |
| `read_color_buffer()` (top row first) | `viewer.color_buffer().read_pixels()`: (H, W, 4) `uint8`, **bottom row first**. |
| `glk::DelayedConstructor(factory)` with the async viewer | `async_viewer.emplace_drawable(name, glk.ThinLines, vertices, line_strip=True, shader_setting=...)`. |
| `spin()` blocks the calling thread | Same, but the GIL is released while rendering (Python worker threads keep running) and Ctrl-C raises `KeyboardInterrupt` out of `spin()`. The GIL does not make viewer calls from worker threads safe: use `invoke()`. |
| Exceptions | An exception in a Python UI callback propagates out of `spin_once()`. |

Not bound in Python: `guik::GLCanvas`, `ProgressModal`, the spdlog sink, `Trajectory`, `PointCorrespondences`, PCL/OpenCV helpers, and ImPlot flag constants (use the integer values listed in `src/example/07_light_viewer_plots.py`).

## Checklist for visualization code

1. Viewer created once on the thread that spins it; no `glk` construction or `viewer` calls from other threads except through `invoke()` (or the async viewer).
2. The loop condition is `spin_once()` (or the code ends in `spin()` / `async_wait()`); otherwise the window never appears or closes immediately.
3. Stable, prefixed drawable names; drawables that stop being valid are removed, and per-item names do not grow without bound.
4. Setting matches data: `VertexColor` only with per-vertex colors, `TextureColor` for grid maps, color ranges matched to the data, point size chosen for the scene scale.
5. C++: exact types in `ShaderSetting::add` (`2.0f`, `Vector4i`), captures in stored callbacks outlive the viewer, `Begin`/`End` balanced.
6. Python: `(changed, value)` unpacked for every editing widget, keyword arguments where the stubs require them.

## Working on this repository

```bash
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=ON        # also: BUILD_PYTHON_BINDINGS, BUILD_EXT_TESTS
make -j$(nproc) && ./01_light_viewer_basic
pip install .                       # Python package (scikit-build-core); scripts/gen_python_stubs.sh regenerates the .pyi stubs
```

- Runtime data (shaders, models) is found through `glk::get_data_path()`, which falls back to the source-tree `data/` directory, so examples run from the build tree without installing.
- When a public C++ API changes, update the binding in `src/python/`, regenerate the stubs, keep the paired `.cpp` / `.py` examples in sync, and update this file and `src/python/pyridescence/AGENTS.md`.
