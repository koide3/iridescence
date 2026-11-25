# pyridescence: guide for coding agents

`pyridescence` is the Python binding of Iridescence, an OpenGL/Dear ImGui viewer for 3D data (point clouds, trajectories, maps, plots, images). This file describes how to write visualization code with it. It is installed next to the package, so every path below is relative to this file.

| Path | Content |
|---|---|
| `guik.pyi`, `glk/__init__.pyi`, `imgui.pyi`, `pfd.pyi`, `gl.pyi` | Type stubs with docstrings. **The authoritative list of functions and signatures**; grep them instead of guessing a name. |
| `examples/*.py` | Runnable examples (`src/example/` in the source repository). Start from the closest one. |
| `data/` | Shaders and models used at runtime (`glk.get_data_path()`). |

Modules: `guik` (viewer, shader settings, cameras), `glk` (drawables, textures, colormaps, PLY IO), `imgui` (Dear ImGui), `pfd` (native file dialogs), `gl` (a few OpenGL constants).

## Mental model

- There is **one viewer window per process**: `guik.viewer()` (same as `guik.LightViewer.instance()`) creates it on the first call and returns the same object afterwards. The constructor arguments (`size`, `background`, `title`) are used only by the first call.
- Things to draw are **drawables registered under unique string names**. `update_*(name, ...)` creates the drawable or replaces the one with the same name; `remove_drawable(name)` deletes it. There is no scene graph.
- Every drawable comes with a **`ShaderSetting`** that holds its color scheme, model matrix (4x4 pose), and point size.
- The UI is immediate-mode: a **UI callback** registered with `register_ui_callback(name, fn)` runs every frame and calls `imgui.*`.
- Nothing is drawn until the viewer **spins**: `spin_once()` renders one frame and returns `False` once the window is closed; `spin()` blocks until then.

```python
import numpy as np
from pyridescence import glk, guik, imgui

viewer = guik.viewer()
points = np.random.randn(10000, 3).astype(np.float32)
point_size = 0.05


def ui():
  global point_size
  imgui.begin('control', None, imgui.WindowFlags_AlwaysAutoResize)
  changed, point_size = imgui.drag_float('point size [m]', point_size, 0.001, 0.001, 1.0)
  if changed:
    viewer.set_point_shape(point_size, True, True)  # (size, metric, circle) for all points
  if imgui.button('close'):
    viewer.close()
  imgui.end()


viewer.register_ui_callback('ui', ui)
viewer.set_point_shape(point_size, True, True)
viewer.update_points('points', points, guik.Rainbow())                    # color = height (Z)
viewer.update_coord('origin', guik.VertexColor().scale(0.5))              # XYZ axes in RGB
viewer.update_thin_lines('path', points[:50], line_strip=True, shader_setting=guik.FlatGreen())

while viewer.spin_once():
  pass  # update drawables here
```

## Which run loop to use

| Situation | Use |
|---|---|
| Static scene, or everything happens in UI callbacks | `viewer.spin()` |
| A processing loop that also draws (the common case) | `while viewer.spin_once(): step(); viewer.update_...()` |
| A long blocking computation that should show intermediate results | Call `viewer.spin_once()` inside it, or use the async viewer |
| Script-like code where the main thread must not care about the viewer | `guik.async_viewer()` (see below) |
| Your own worker `threading.Thread` | `viewer.invoke(fn)` from the worker, `spin_once()` loop on the main thread |

Rules that follow from OpenGL being single-threaded:

- Create the viewer on the main thread, and construct drawables (`glk.PointCloudBuffer`, `glk.ThinLines`, `glk.create_texture`, ...) and call `viewer.*` only on that thread. Doing it elsewhere crashes instead of raising.
- From another thread, wrap the work: `viewer.invoke(lambda: viewer.update_points(...))`. It runs on the viewer thread before the next frame. Bind loop variables as default arguments (`def task(i=i, pts=pts)`). Only `append_text()` / `clear_text()` are safe to call directly.
- `spin()` and `spin_once()` release the GIL while rendering, so Python worker threads keep running, and `spin()` raises `KeyboardInterrupt` on Ctrl-C. The flip side: the GIL does **not** protect the viewer. A worker thread that calls viewer methods directly (even non-OpenGL ones such as `register_ui_callback` or `update_plot_line`) races with the rendering thread; use `invoke()`.
- An exception raised in a UI callback propagates out of `spin_once()` / `spin()`.

### Async viewer

`guik.async_viewer()` runs the viewer in a background thread and offers thread-safe versions of most `update_*`, camera, plot, and image methods, which copy the data and return immediately.

```python
viewer = guik.async_viewer()
viewer.update_points('map', points, guik.Rainbow())
viewer.emplace_drawable('line', glk.ThinLines, vertices, line_strip=True, shader_setting=guik.FlatOrange())
viewer.register_ui_callback('ui', ui)   # runs on the viewer thread; guik.viewer() is safe to use in it
guik.async_wait()                       # block until the window is closed; guik.async_running() polls
```

- Do not construct `glk` drawables on the main thread: use `emplace_drawable(name, cls, *args, shader_setting=...)` or `glk.DelayedConstructor(cls, *args)`, which build the object on the viewer thread.
- Do not call `guik.viewer()` from the main thread while the async viewer is alive.
- Not available on it: `setup_plot`, `invoke`, `find_drawable`, picking, buffer readback, and return values (`update_points` returns `None`, `save_color_buffer` is deferred). `update_image` takes `(name, width, height, rgba_bytes)` instead of a texture. Use the synchronous viewer when these are needed.

## Drawables

Shorthands on the viewer (and on sub-viewers) cover most needs:

| Call | Notes |
|---|---|
| `update_points(name, points, shader_setting)` | `points`: (N, 3) or (N, 4). Returns the `glk.PointCloudBuffer`. |
| `update_points(name, points, colors=, intensities=, cmaps=, normals=, intensity_colormap=, shader_setting=)` | Per-point attributes, see "Coloring". |
| `update_thin_lines(name, vertices, colors=, indices=, line_strip=False, line_width=1.0, shader_setting=)` | `line_strip=True`: polyline through all vertices (trajectories). `False`: independent segments from vertex pairs. `shader_setting` **must be a keyword argument**. Line width is in pixels. |
| `update_coord(name, setting)` | Coordinate frame; needs `guik.VertexColor(pose)` to show RGB axes. |
| `update_sphere / cube / cone / icosahedron / frustum`, `update_wire_*` | Unit primitives placed and sized by the model matrix of the setting. |
| `update_normal_dists(name, means, covs, sigma, setting)` | Ellipsoids of Gaussians. |
| `update_drawable(name, drawable, setting)` | Any `glk` drawable below. |

`glk` drawables for `update_drawable`:

- `glk.primitives.sphere()`, `cube()`, `cone()`, `icosahedron()`, `bunny()`, `frustum()`, `coordinate_system()` and `wire_*()`. They are cached singletons, cheap to pass every frame.
- `glk.PointCloudBuffer(points)` with `add_color`, `add_intensity(colormap, values)`, `add_colormap(values)`, `add_normals`, `add_buffer(attribute_name, data)`. `glk.IndexedPointCloudBuffer(buffer, indices)` draws a subset without copying.
- `glk.ThinLines(points, [colors], line_strip=, line_width=)` and `glk.Lines(x=, y=, z=, c=, width=)` (thick lines, width in meters). `glk.PointNormalsBuffer(points, normals, length)`.
- `glk.Mesh(vertices, indices, normals=, colors=, tex_coords=, wireframe=)`.
- Maps: `glk.VoxelMap(int_voxel_coords, resolution, glk.MeshRenderingOptions())` (voxel `c` covers `[c, c + 1] * resolution`), `glk.GridMap(resolution, values, alpha, mode, scale)` (2D textured grid, `values[y, x]`, draw with `guik.TextureColor`), `glk.ElevationMap(glk.ElevationMapBuilder(resolution, (width, height), origin_xy))` (height surface; see `examples/02_light_viewer_maps.py`).
- `glk.DrawableContainer([drawables])` groups drawables under one name.
- Subclass `glk.Drawable` and implement `draw(self, shader)` for custom OpenGL rendering (`examples/py_custom_drawable.py`, `py_custom_shader.py`; requires PyOpenGL and `super().__init__()`).

Array arguments accept any array-like and are converted to `float32` (`int32` for voxel coordinates). Pass C-contiguous `float32` arrays to avoid a copy on large data. Filter out NaN/inf points yourself.

### Naming and lifetime

- Name drawables with a group prefix and an ID: `f'frame_{i}'`, `f'submap_{id}'`, `'map_occupancy'`. Prefixes make group operations one-liners:
  - `viewer.remove_drawable('frame_.*', True)`: regex removal. The pattern must match the **whole** name (`'frame'` removes nothing but a drawable named exactly `frame`).
  - `viewer.register_drawable_filter('f', lambda name: show_frames or not name.startswith('frame_'))`: hide a group without deleting it. This is the right tool for "show X" checkboxes over data that is expensive to rebuild.
- `remove_drawable` on a missing name is a no-op. `clear_drawables()` removes everything.
- `setting, drawable = viewer.find_drawable(name)` returns `(None, None)` if missing. Mutating the returned setting (`set_model_matrix`, `set_color`, `set_alpha`, ...) takes effect immediately without re-uploading geometry. Use it to move or recolor existing objects.
- `guik.anon()` returns a fresh unique name for throwaway drawables.

## ShaderSetting: color, pose, point size

### Coloring

| Setting | Color comes from |
|---|---|
| `guik.Rainbow()` | Position along Z (default range -3 to 5 m), through the viewer colormap. |
| `guik.FlatColor(r, g, b, a)`, `FlatRed/Green/Blue/Orange/White/Gray/DarkGray/Black()` | One color. All four components are required, in [0, 1]. |
| `guik.VertexColor()` | Per-vertex colors of the drawable: `colors=` (N x 3 or N x 4 in [0, 1]) or `intensities=` (N values in [0, 1], mapped with `intensity_colormap=` on the CPU) of `update_points`, colors of lines and meshes, axes of `update_coord`. |
| `guik.TextureColor()` | Texture of the drawable (`glk.GridMap`, textured meshes). |
| `guik.VertexColorMap()` | Per-vertex scalar (`cmaps=` of `update_points`, color layer of `glk.ElevationMap`) mapped on the GPU with the viewer colormap over `cmap_range`. |

- A setting whose mode has no matching data renders black or wrong: `VertexColor` on a cloud without colors, or `FlatColor` on `update_coord` (gives single-colored axes).
- Ranges are uniforms: `.add('z_range', np.float32([zmin, zmax]))` for `Rainbow`, `.add('cmap_range', np.float32([vmin, vmax]))` for `VertexColorMap`. Set them per drawable, or for everything with `viewer.shader_setting().add(...)`. Geometry outside the Z range saturates to one color, so set `z_range` to match the data.
- `viewer.set_colormap(glk.COLORMAP.TURBO)` selects the colormap (list: `glk.COLORMAP.__members__`). `glk.colormap(cmap, 0..255)` returns RGBA `uint8` and `glk.colormapf(cmap, 0.0..1.0)` RGBA floats, for building per-point colors or lookup tables yourself.
- Alpha < 0.999 (`FlatColor(..., 0.5)`, `set_alpha(0.5)`) switches the drawable to the transparent pass. For settings without a flat color (`TextureColor`, `VertexColor`, `Rainbow`), call `.make_transparent()` explicitly.
- `ShaderSetting.add(name, value)` only takes a float, a 2-vector, or a 4x4 matrix; `addi(name, int4)` takes integer 4-vectors. The setters return the setting, so calls chain.

### Pose

Every setting constructor takes an optional 4x4 model matrix as the last argument: `guik.Rainbow(T_world_sensor)`, `guik.FlatColor(r, g, b, a, T)`, `guik.VertexColor(T)`. Alternatives are the keywords `scale=`, `trans=`, `rot=`.

```python
guik.VertexColor(T_world_body).scale(0.3)          # pose from a matrix, then shrink the axes
guik.FlatRed().translate(x, y, z).scale(0.1)       # small sphere/cube at a point
guik.FlatColor(0.2, 0.9, 0.4, 0.35, T_box)         # T_box = pose @ diag(sx, sy, sz, 1) for a box
```

- `translate()` adds a world-frame offset and `scale()` scales the object about its own origin, in any order. `rotate()` and `transform()` right-multiply the current matrix.
- Keep sensor-frame data in its own frame and put the pose in the model matrix (`update_points('scan', scan_sensor, guik.FlatOrange(T_world_sensor))`) instead of transforming points with numpy. Moving an object is then a matter of replacing the setting.
- The world is right-handed and Z-up by default (the grid is the XY plane). Rotate Y-up data with the model matrix, or use `use_orbit_camera_control_xz()` for camera-style frames.

### Point size

- `viewer.set_point_shape(size, metric, circle)` sets the default for all clouds. `metric=True`: `size` is a radius in meters (`0.02` suits indoor scans, larger for outdoor maps). `metric=False`: size in pixels.
- Per drawable: `.set_point_scale(1.5)` multiplies the default (highlight the current scan over the map), `.set_point_size(0.05)` or `.set_point_shape(size, metric, circle)` overrides it.

## UI (imgui)

- Function names are snake_case versions of Dear ImGui (`imgui.same_line()`, `imgui.begin_main_menu_bar()`); constants keep their names without the `ImGui` prefix (`imgui.WindowFlags_AlwaysAutoResize`, `imgui.Cond_FirstUseEver`).
- Widgets that edit a value return `(changed, value)` instead of writing through a pointer. Assign the value back and act on `changed`:
  `changed, self.speed = imgui.slider_float('speed', self.speed, 0.1, 10.0)`. Buttons and menu items return a `bool`.
- Callbacks have no arguments and no retained state: keep state in an object (`viewer.register_ui_callback('ui', self._ui)`) or in globals.
- Every `imgui.begin()` needs `imgui.end()`, also on early returns. `begin_menu`, `begin_popup`, `begin_table`, `begin_combo` need their `end_*` only when they returned `True`.
- Do cheap viewer updates directly in the callback (it runs on the viewer thread). For heavy work, set a flag in the callback and do the work in the main loop, so the UI keeps responding.
- Vectors and colors are numpy arrays (`imgui.text_colored(np.float32([1, 0.5, 0.1, 1]), 'text')`, `imgui.color_edit4`, `imgui.drag_float_range2`). 2D overlay colors are packed with `imgui.IM_COL32(r, g, b, a)`.
- `imgui.show_demo_window()` shows every widget; search `imgui.pyi` for the bound subset.
- Ready-made helpers: `viewer.append_text(str)` (log window), `guik.ModelControl(name)` (pose gizmo, `model_matrix()`), `guik.HoveredDrawings()` (text and markers pinned to 3D points or drawables; register `hovered.create_callback()` as a UI callback), `pfd.open_file / save_file / select_folder` (blocking native dialogs returning `''` on cancel), `guik.RecentFiles(tag)`.

## Camera

- `use_orbit_camera_control(distance, theta, phi)` (default; Z-up, angles in radians), `use_orbit_camera_control_xz`, `use_topdown_camera_control(distance, theta)`, `use_arcball_camera_control`, `use_fps_camera_control(fovy_deg)`, `use_sensor_view_camera_control(T_sensor_camera)` (chase camera; call `set_sensor_pose(T_world_sensor)` on the returned object every frame).
- `viewer.lookat(xyz)` moves the camera center; call it every frame behind a "follow" checkbox to track a moving sensor. `reset_center()` returns to the origin.
- Set `distance` to the scale of the scene (a few meters for a single scan, tens of meters for a map). The far plane is 1000 by default: `use_perspective_projection_control(fovy_deg, near, far)`, `use_orthographic_projection_control(width)`.
- `disable_xy_grid()` hides the ground grid; `set_clear_color([r, g, b, a])` sets the background.
- `save_camera_settings(path)` / `load_camera_settings(path)` persist the view. `get_camera_control().view_matrix()` returns the 4x4 view matrix.

## Plots and images

```python
viewer.setup_plot('position [m]', 520, 170)          # optional: size and ImPlot flags
viewer.update_plot_line('position [m]', 'x', ts, xs)  # (plot, label, xs, ys) or (plot, label, ys)
viewer.fit_plot('position [m]')                       # refit the axes to the data
viewer.update_image('range image', glk.create_texture(image_u8), 2.0)
```

- A plot/label or image name identifies the item; updating with the same names replaces the data. The whole series is passed every time, so cap the history for real-time plots (`collections.deque(maxlen=N)` or `xs[-N:]`).
- Axes do not follow new data on their own: call `fit_plot(name)` / `fit_all_plots()` after updating, or pass ImPlot auto-fit axis flags to `setup_plot` (`examples/07_light_viewer_plots.py` lists the flag values, which are not exported as constants).
- A name of the form `'group/name'` puts plots (and images) with the same group in one tabbed window.
- Also: `update_plot_scatter`, `update_plot_stairs`, `update_plot_histogram`, `link_plot_axes`, `remove_plot`, `clear_plots`.
- `glk.create_texture()` takes `uint8` images of shape (H, W) or (H, W, C) with channels in **BGR(A)** order (OpenCV convention); `create_texture_f()` takes `float32`. `remove_image(name)` closes one.
- `viewer.sub_viewer(name, (w, h))` opens a second 3D view in its own window with the same drawable and camera API, for showing a different scene (a local map, a calibration rig) next to the main one.

## Picking and capture

- In a UI callback: `pt = viewer.pick_point(1)` returns the 3D point under a right click, or `None`. Lower level: `pick_depth(pixel)` (1.0 = background) and `unproject(pixel, depth)`; guard manual mouse handling with `if imgui.get_io().want_capture_mouse: return`.
- To identify the clicked object, call `viewer.enable_info_buffer()`, tag settings with `.addi('info_values', np.int32([id, 0, 0, 0]))`, and read `viewer.pick_info(pixel)` (`[-1, -1, -1, -1]` on the background). See `examples/04_light_viewer_picking.py`.
- `viewer.save_color_buffer('shot.png')` saves the last rendered frame; `viewer.color_buffer().read_pixels()` returns it as an (H, W, 4) `uint8` RGBA array and `viewer.depth_buffer().read_pixels(gl.DEPTH_COMPONENT, gl.FLOAT)` as (H, W) `float32`. Rows are **bottom-up**: flip with `[::-1]`. The buffers hold the frame of the last `spin_once()`, so spin after the last update and before reading.
- Offscreen rendering (videos, tests): `guik.viewer(size=np.array([1280, 720]), background=True)` creates a hidden window; then loop over update, `spin_once()`, read. It still needs an OpenGL context, i.e. a display (use `xvfb-run` on headless machines).

## Performance and pacing

- `spin_once()` waits for vsync (the display refresh rate) and can block entirely while the window is hidden or the screen is locked. For processing loops call `viewer.disable_vsync()` once, and limit the frame rate yourself if needed.
- Do not spin once per processed item when processing is faster than the display: spin when 1/30 to 1/60 s has passed since the last frame. For real-time playback of a log, spin in a loop until the wall clock catches up with the data timestamp, which also keeps the UI responsive while paused.
- Each `update_points` uploads a buffer. Re-upload large, slowly changing clouds (accumulated maps) every N frames or a few times per second; update small things (current scan, pose, trajectory) every frame.
- When only the pose or color changes, replace or mutate the setting (`find_drawable`) and reuse the drawable. To reuse one buffer under several names, keep the object returned by `update_points` and pass it to `update_drawable`.
- Downsample for display (voxel grid or `points[::k]`) once clouds reach tens of millions of points.
- Screen-space effects: `viewer.set_screen_effect(glk.ScreenSpaceAmbientOcclusion())` adds depth cues to dense clouds; `glk.ScreenSpaceLighting()` additionally needs `viewer.enable_normal_buffer()` and per-point normals.

## Checklist for visualization code

1. Viewer created once, on the main thread; no `glk` or `viewer` calls from other threads except through `invoke()` (or the async viewer).
2. The loop condition is `viewer.spin_once()` (or the code ends in `spin()` / `guik.async_wait()`); otherwise the window never appears or closes immediately.
3. Stable, prefixed drawable names; drawables that stop being valid are removed, and per-item names (`f'frame_{i}'`) do not grow without bound.
4. Setting matches data: `VertexColor` only with per-vertex colors, `TextureColor` for grid maps, `z_range` / `cmap_range` matched to the data, point size chosen for the scene scale.
5. `(changed, value)` unpacked for every editing widget, and `begin`/`end` balanced.
6. Signatures taken from the `.pyi` stubs or an example, not from the C++ API or memory: the Python bindings cover a subset and differ in details (keyword arguments, tuple returns).
