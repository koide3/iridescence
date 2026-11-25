#!/usr/bin/python3
import numpy
from pyridescence import *

viewer = guik.LightViewer.instance()
# enable information buffer to recognize picked boxes
viewer.enable_info_buffer()

for x in range(-5, 6):
  for y in range(-5, 6):
    transform = numpy.diag([0.5, 0.5, 0.5, 1.0]).astype(numpy.float32)
    transform[:3, 3] = [x, y, 0.0]

    # draw boxes with unique IDs
    shader_setting = guik.FlatColor((x + 5) / 10.0, (y + 5) / 10.0, 1.0, 1.0, transform).addi('info_values', numpy.array([x, y, 0, 0], dtype=numpy.int32))
    viewer.update_drawable(f'cube_{x}_{y}', glk.primitives.cube(), shader_setting)


def picking():
  if imgui.get_io().want_capture_mouse:
    return

  if imgui.is_mouse_clicked(1):
    mouse_pos = imgui.get_mouse_pos().astype(numpy.int32)
    info = viewer.pick_info(mouse_pos)
    print('info:', info)

    if info[3] == -1:
      return

    # get the depth and 3d coordinate of the clicked position
    depth = viewer.pick_depth(mouse_pos)
    pos = viewer.unproject(mouse_pos, depth)
    print('depth:', depth, 'pos:', pos)

    # change the picked cube's color
    shader_setting, _ = viewer.find_drawable(f'cube_{info[0]}_{info[1]}')
    if shader_setting is not None:
      shader_setting.set_color_mode(1).set_color(1.0, 0.0, 0.0, 1.0)  # 1: FLAT_COLOR


viewer.register_ui_callback('picking', picking)

viewer.spin()
