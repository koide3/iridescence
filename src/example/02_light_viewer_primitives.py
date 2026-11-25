#!/usr/bin/python3
import numpy
from pyridescence import *


def translation(x, y, z):
  T = numpy.identity(4, dtype=numpy.float32)
  T[:3, 3] = [x, y, z]
  return T


def rotation_x(angle):
  T = numpy.identity(4, dtype=numpy.float32)
  c, s = numpy.cos(angle), numpy.sin(angle)
  T[1:3, 1:3] = [[c, -s], [s, c]]
  return T


def uniform_scaling(s):
  return numpy.diag([s, s, s, 1.0]).astype(numpy.float32)


def random_rotation():
  # Uniformly random rotation from the QR decomposition of a Gaussian matrix
  q, r = numpy.linalg.qr(numpy.random.randn(3, 3))
  q = q @ numpy.diag(numpy.sign(numpy.diag(r)))
  if numpy.linalg.det(q) < 0:
    q[:, 0] = -q[:, 0]
  T = numpy.identity(4, dtype=numpy.float32)
  T[:3, :3] = q
  return T


viewer = guik.LightViewer.instance()

# sine wave
xs = numpy.arange(-5.0, 5.0 + 1e-6, 0.1)
line_vertices = numpy.stack([xs, numpy.cos(xs), numpy.ones_like(xs)], axis=-1).astype(numpy.float32)

# thin lines (GL_LINES)
viewer.update_drawable('thin_lines', glk.ThinLines(line_vertices, line_strip=True), guik.FlatColor(0.0, 1.0, 0.0, 1.0))

# thick lines with flat and vertex colors
# The pyplot-like constructor creates a line strip colored with the TURBO colormap of c
lines = glk.Lines(x=line_vertices[:, 0], y=line_vertices[:, 1], z=line_vertices[:, 2], c=(xs + 5.0) / 10.0, width=0.1)
viewer.update_drawable('lines', lines, guik.FlatColor(0.0, 1.0, 0.0, 1.0).translate(0.0, 2.0, 0.0))
viewer.update_drawable('colored_lines', lines, guik.VertexColor().translate(0.0, 4.0, 0.0))

# coordinate systems
draw_coords = True


def coord_rendering_setting():
  global draw_coords
  _, draw_coords = imgui.checkbox('coords', draw_coords)


viewer.register_ui_callback('coord_rendering_setting', coord_rendering_setting)
viewer.register_drawable_filter('coord_filter', lambda drawable_name: draw_coords or 'coord_' not in drawable_name)

for x in numpy.arange(-5.0, 5.0 + 1e-6, 2.0):
  transform = translation(x, 6.0, 1.0) @ random_rotation()
  viewer.update_drawable(f'coord_{x}', glk.primitives.coordinate_system(), guik.VertexColor(transform))

# bunny transform (translation * rotation * scaling)
def bunny_transform(y):
  return translation(0.0, y, 0.0) @ rotation_x(numpy.pi / 2) @ uniform_scaling(8.0)


# primitives
viewer.update_drawable('solid_icosahedron', glk.primitives.icosahedron(), guik.Rainbow().scale(0.5).translate(-5.0, 9.0, 1.0))
viewer.update_drawable('solid_sphere', glk.primitives.sphere(), guik.Rainbow().translate(-2.5, 9.0, 1.0))
viewer.update_drawable('solid_bunny', glk.primitives.bunny(), guik.Rainbow(bunny_transform(9.0)))
viewer.update_drawable('solid_cube', glk.primitives.cube(), guik.Rainbow().translate(2.5, 9.0, 1.0))
viewer.update_drawable('solid_cone', glk.primitives.cone(), guik.Rainbow().translate(5.0, 9.0, 1.0))

# wireframes
viewer.update_drawable('wire_icosahedron', glk.primitives.wire_icosahedron(), guik.Rainbow().scale(0.5).translate(-5.0, 12.0, 1.0))
viewer.update_drawable('wire_sphere', glk.primitives.wire_sphere(), guik.Rainbow().translate(-2.5, 12.0, 1.0))
viewer.update_drawable('wire_bunny', glk.primitives.wire_bunny(), guik.Rainbow(bunny_transform(12.0)))
viewer.update_drawable('wire_cube', glk.primitives.wire_cube(), guik.Rainbow().translate(2.5, 12.0, 1.0))
viewer.update_drawable('wire_cone', glk.primitives.wire_cone(), guik.Rainbow().translate(5.0, 12.0, 1.0))

# transparent
viewer.update_drawable('trans_icosahedron', glk.primitives.icosahedron(), guik.FlatColor(1.0, 0.5, 0.0, 0.5).scale(0.5).translate(-5.0, 15.0, 1.0).make_transparent())
viewer.update_drawable('trans_sphere', glk.primitives.sphere(), guik.FlatColor(1.0, 0.5, 0.0, 0.5).translate(-2.5, 15.0, 1.0).make_transparent())
viewer.update_drawable('trans_bunny', glk.primitives.bunny(), guik.FlatColor(1.0, 0.5, 0.0, 0.5, bunny_transform(15.0)).make_transparent())
viewer.update_drawable('trans_cube', glk.primitives.cube(), guik.FlatColor(1.0, 0.5, 0.0, 0.5).translate(2.5, 15.0, 1.0).make_transparent())
viewer.update_drawable('trans_cone', glk.primitives.cone(), guik.FlatColor(1.0, 0.5, 0.0, 0.5).translate(5.0, 15.0, 1.0).make_transparent())

# rendering of drawables can be selectively turned off
switches = {'lines': True, 'primitives': True, 'wireframes': True, 'transparent': True}


def rendering_switch():
  for key in switches:
    _, switches[key] = imgui.checkbox(key, switches[key])


def drawable_filter(drawable_name):
  if not switches['lines'] and 'line' in drawable_name:
    return False
  if not switches['primitives'] and 'solid_' in drawable_name:
    return False
  if not switches['wireframes'] and 'wire_' in drawable_name:
    return False
  if not switches['transparent'] and 'trans_' in drawable_name:
    return False
  return True


viewer.register_ui_callback('rendering_switch', rendering_switch)
viewer.register_drawable_filter('drawable_filter', drawable_filter)

# hovered 2D drawings
hovered = guik.HoveredDrawings()
viewer.register_ui_callback('hovered', hovered.create_callback())

white = imgui.IM_COL32(255, 255, 255, 255)
black_bg = imgui.IM_COL32(0, 0, 0, 128)

# Put hovered texts
hovered.add_text([8.0, 0.0, 1.0], 'thin_lines', white, black_bg)
hovered.add_text([8.0, 3.0, 1.0], 'lines')
hovered.add_text([8.0, 6.0, 1.0], 'coords')
hovered.add_text([8.0, 9.0, 1.0], 'solid')
hovered.add_text([8.0, 12.0, 1.0], 'wire')
hovered.add_text([8.0, 15.0, 1.0], 'transparent')

# Put texts on drawables
hovered.add_text_on('trans_icosahedron', 'icosahedron')
hovered.add_text_on('trans_sphere', 'sphere')
hovered.add_text_on('trans_bunny', 'bunny')
hovered.add_text_on('trans_cube', 'cube')
hovered.add_text_on('trans_cone', 'cone')

# Put shapes at 3D points
hovered.add_circle([-5.0, 18.0, 1.0], white, 12.0)
hovered.add_cross([-2.5, 18.0, 1.0], white, 10.0)
hovered.add_triangle([0.0, 18.0, 1.0], white, 20.0, 1.0, False, True)
hovered.add_rect([2.5, 18.0, 1.0], white, [20.0, 20.0])
hovered.add_filled_rect([5.0, 18.0, 1.0], white, [20.0, 20.0])

hovered.add_text([-5.0, 18.0, 1.0], 'circle', white, black_bg, [0.0, 25.0])
hovered.add_text([-2.5, 18.0, 1.0], 'cross', white, black_bg, [0.0, 25.0])
hovered.add_text([0.0, 18.0, 1.0], 'triangle', white, black_bg, [0.0, 25.0])
hovered.add_text([2.5, 18.0, 1.0], 'rect', white, black_bg, [0.0, 25.0])
hovered.add_text([5.0, 18.0, 1.0], 'filled_rect', white, black_bg, [0.0, 25.0])

# Change the coloring bandwidth of the guik.Rainbow coloring scheme
z_range = numpy.array([-3.0, 5.0], dtype=numpy.float32)


def control_z_range():
  global z_range
  changed, z_range = imgui.drag_float_range2('z_range', z_range, 0.01)
  if changed:
    viewer.shader_setting().add('z_range', z_range)


viewer.register_ui_callback('control_z_range', control_z_range)

viewer.spin()
