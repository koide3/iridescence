#!/usr/bin/python3
import numpy
import scipy.spatial
from pyridescence import *

viewer = guik.LightViewer.instance()
viewer.enable_normal_buffer()
viewer.set_screen_effect(glk.ScreenSpaceLighting())

sub_viewer = viewer.sub_viewer('sub')
sub_viewer.update_drawable('cube', glk.primitives.cube(), guik.Rainbow(scale=2.0, trans=(1.0, 0.0, 0.0)))

cloud = numpy.random.randn(8192, 3)
cloud = 5.0 * (cloud.T / numpy.linalg.norm(cloud, axis=1)).T
cloud_buffer = glk.create_pointcloud_buffer(cloud)
viewer.update_drawable('cloud', cloud_buffer, guik.Rainbow().add('point_scale', 2.0).make_transparent())
viewer.update_drawable('icosahedron', glk.primitives.icosahedron(), guik.FlatColor(1.0, 0.5, 0.0, 0.5, scale=2.0))

time = 0.0
def callback():
  imgui.begin('ui', None, imgui.WindowFlags_AlwaysAutoResize)

  global time
  updated, time = imgui.drag_float('time', time, 0.1)
  if updated:
    viewer.append_text('time:%.3f' % time)
    setting, drawable = viewer.find_drawable('cloud')
    matrix = numpy.identity(4)
    matrix[:3, :3] = scipy.spatial.transform.Rotation.from_rotvec([0, 0, time]).as_matrix()
    setting.add('model_matrix', matrix)
  
  if imgui.button('close'):
    exit(0)

  imgui.end()

viewer.register_ui_callback('callback', callback)

viewer.spin()