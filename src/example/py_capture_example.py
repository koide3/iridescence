#!/usr/bin/python3
# Capturing the rendering results as numpy arrays, saving them, and showing images on the viewer
import numpy
from pyridescence import *

viewer = guik.viewer()
viewer.disable_xy_grid()
viewer.use_orbit_camera_control(25.0)  # Camera distance

output_prefix = '/tmp/iridescence_capture'
live_capture = True
color = None  # (height, width, 4) uint8 RGBA
depth = None  # (height, width) float32 in [0, 1] (1.0 = background)


def capture():
  # The frame buffers of the viewer are textures, and their contents can be read as numpy arrays.
  # Note that the first row of the read array is the bottom row of the screen, so the arrays are flipped vertically here.
  # This must be called in the viewer thread (the main thread here) after at least one frame is rendered.
  global color, depth
  color = numpy.flipud(viewer.color_buffer().read_pixels())  # gl.RGBA and gl.UNSIGNED_BYTE by default
  depth = numpy.flipud(viewer.depth_buffer().read_pixels(gl.DEPTH_COMPONENT, gl.FLOAT))


def update_images():
  # Show processed images on the viewer.
  # glk.create_texture() takes (height, width) or (height, width, channels) uint8 images. Color channels are in the BGR(A) order (OpenCV style).
  step = 4  # Downsampling
  small_color = color[::step, ::step]
  small_depth = depth[::step, ::step]

  # Edges of the color image (simple gradient magnitude)
  gray = small_color[:, :, :3].astype(numpy.float32).mean(axis=2)
  grad_y, grad_x = numpy.gradient(gray)
  edges = numpy.clip(numpy.hypot(grad_x, grad_y) * 4.0, 0, 255).astype(numpy.uint8)
  viewer.update_image('edges', glk.create_texture(edges))

  # Depth image colorized with a colormap (near = red, far = blue, background = black)
  foreground = small_depth < 1.0
  depth_image = numpy.zeros(small_depth.shape + (4,), dtype=numpy.uint8)
  if foreground.any():
    near, far = small_depth[foreground].min(), small_depth[foreground].max()
    normalized = 255 - (255 * (small_depth - near) / max(far - near, 1e-6)).clip(0, 255).astype(numpy.uint8)
    table = numpy.array([glk.colormap(glk.COLORMAP.TURBO, i) for i in range(256)], dtype=numpy.uint8)  # 256 x 4 (RGBA)
    depth_image[foreground] = table[normalized[foreground]][:, [2, 1, 0, 3]]  # RGBA -> BGRA
    depth_image[foreground, 3] = 255
  viewer.update_image('depth', glk.create_texture(depth_image))


def ui_callback():
  global live_capture
  imgui.begin('capture', True, imgui.WindowFlags_AlwaysAutoResize)
  _, live_capture = imgui.checkbox('live capture', live_capture)

  if color is not None:
    imgui.text(f'color: {color.shape} {color.dtype}')
    imgui.text(f'depth: {depth.shape} {depth.dtype} [{depth.min():.4f}, {depth.max():.4f}]')

  # Save the buffers as PNG images with the viewer functions
  if imgui.button('save PNG'):
    viewer.save_color_buffer(output_prefix + '_color.png')
    viewer.save_depth_buffer(output_prefix + '_depth.png')
    viewer.append_text(f'saved to {output_prefix}_color.png and {output_prefix}_depth.png')

  # Save the captured numpy arrays
  imgui.same_line()
  if imgui.button('save numpy') and color is not None:
    numpy.savez_compressed(output_prefix + '.npz', color=color, depth=depth)
    viewer.append_text(f'saved to {output_prefix}.npz')
  imgui.end()


viewer.register_ui_callback('ui', ui_callback)

angle = 0.0
frame = 0
while viewer.spin_once():
  # Rotating bunny
  angle += imgui.get_io().delta_time * 0.5
  c, s = numpy.cos(angle), numpy.sin(angle)
  rot_z = numpy.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=numpy.float32)
  y_up_to_z_up = numpy.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=numpy.float32)
  bunny_transform = rot_z @ y_up_to_z_up @ numpy.diag([50.0, 50.0, 50.0, 1.0]).astype(numpy.float32)
  bunny_transform[2, 3] = -5.0  # Center the bunny vertically
  viewer.update_drawable('bunny', glk.primitives.bunny(), guik.Rainbow(bunny_transform))

  # Capture the frame rendered by the last spin_once() every few frames
  frame += 1
  if live_capture and frame % 5 == 0:
    capture()
    update_images()
