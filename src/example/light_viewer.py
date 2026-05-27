#!/usr/bin/python3
import numpy
from pyridescence import *

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
    matrix[:2, :2] = [[numpy.cos(time), -numpy.sin(time)], [numpy.sin(time), numpy.cos(time)]]
    setting.add('model_matrix', matrix)

  global text
  imgui.separator()
  _, text = imgui.input_text('text', text)

  imgui.separator()
  if imgui.button('close'):
    exit(0)

  imgui.end()

viewer.register_ui_callback('callback', callback)

viewer.spin()