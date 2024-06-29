#!/usr/bin/python3
import numpy
import scipy.spatial
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
for i in range(5):
  # Objects to be rendered are called "drawables" and managed with unique names.
  # Here, solid and wire spheres are registered to the viewer respectively with the "Rainbow" and "FlatColor" coloring schemes.
  # The "Rainbow" coloring scheme encodes the height of each fragment using the turbo colormap by default.
  transform = numpy.identity(4)
  transform[:3, :3] = Rotation.from_rotvec([0.0, 0.0, angle]).as_matrix()
  viewer.update_drawable('sphere', glk.primitives.sphere(), guik.Rainbow(transform))
  viewer.update_drawable('wire_sphere', glk.primitives.wire_sphere(), guik.FlatColor(0.1, 0.7, 1.0, 1.0, transform))
  viewer.spin_once()


viewer = guik.LightViewer.instance()
viewer.enable_normal_buffer()
viewer.set_screen_effect(glk.ScreenSpaceLighting())

sub_viewer = viewer.sub_viewer('sub')
sub_viewer.update_drawable('cube', glk.primitives.cube(), guik.Rainbow(scale=2.0, trans=(1.0, 0.0, 0.0)))

def sub_ui_callback():
  io = imgui.get_io()
  imgui.button('sub viewer')
sub_viewer.register_ui_callback('ui', sub_ui_callback)

cloud = numpy.random.randn(8192, 3)
cloud = 5.0 * (cloud.T / numpy.linalg.norm(cloud, axis=1)).T
cloud_buffer = glk.create_pointcloud_buffer(cloud)
viewer.update_drawable('cloud', cloud_buffer, guik.Rainbow().add('point_scale', 2.0).make_transparent())
viewer.update_drawable('icosahedron', glk.primitives.icosahedron(), guik.FlatColor(1.0, 0.5, 0.0, 0.5, scale=2.0))

time = 0.0
text = 'test'
model_control = guik.ModelControl('model_control')
def callback():
  imgui.begin('ui', None, imgui.WindowFlags_AlwaysAutoResize)

  io = imgui.get_io()
  if io.mouse_clicked[imgui.MouseButton_Right]:
    winpos = imgui.get_window_pos()
    clicked_pos = io.mouse_pos - winpos
    
    depth = sub_viewer.pick_depth(clicked_pos)
    if depth > 0.0:
      pt = sub_viewer.unproject(clicked_pos, depth)
      model_matrix = numpy.identity(4)
      model_matrix[:3, 3] = pt
      model_matrix[:3, :3] *= 0.1
      sub_viewer.update_drawable('sphere', glk.primitives.sphere(), guik.FlatRed(scale = 0.1, trans = pt))
      print(pt)

    # mouse_pos = imgui.get_mouse_pos()

  model_control.draw_gizmo_ui()
  model_control.draw_gizmo()
  setting, drawable = viewer.find_drawable('icosahedron')
  setting.add('model_matrix', model_control.model_matrix())

  global time
  imgui.separator()
  updated, time = imgui.drag_float('time', time, 0.1)
  if updated:
    viewer.append_text('time:%.3f' % time)
    setting, drawable = viewer.find_drawable('cloud')
    matrix = numpy.identity(4)
    matrix[:3, :3] = scipy.spatial.transform.Rotation.from_rotvec([0, 0, time]).as_matrix()
    setting.add('model_matrix', matrix)

  global text
  imgui.separator()
  _, text = imgui.input_text('text', text)

  imgui.separator()
  if imgui.button('close'):
    exit(0)

  imgui.end()

viewer.register_ui_callback('callback', callback)

viewer.spin_once()
guik.destroy()