#!/usr/bin/python3
import numpy
from pyridescence import *


# Point cloud with per-point attributes
class PointCloud:
  def __init__(self, ply):
    # glk.PLYData holds vertices, normals, colors, intensities, indices (faces), and generic properties
    self.vertices = numpy.array(ply.vertices[:, :3], dtype=numpy.float32)
    num_points = len(self.vertices)

    min_pt, max_pt = self.vertices.min(axis=0), self.vertices.max(axis=0)
    self.center = (min_pt + max_pt) / 2.0  # Center of the bounding box
    self.extent = max(float((max_pt - min_pt).max()), 1e-6)  # Longest edge length of the bounding box

    # Normals: use the ones in the file, or estimate them from the faces if available
    self.normals = None
    if len(ply.normals) == num_points:
      self.normals = numpy.array(ply.normals[:, :3], dtype=numpy.float32)
    elif len(ply.indices):
      triangles = numpy.asarray(ply.indices).reshape(-1, 3)
      v0, v1, v2 = (self.vertices[triangles[:, i]] for i in range(3))
      face_normals = numpy.cross(v1 - v0, v2 - v0)
      normals = numpy.zeros_like(self.vertices)
      for i in range(3):
        numpy.add.at(normals, triangles[:, i], face_normals)
      norms = numpy.linalg.norm(normals, axis=1, keepdims=True)
      self.normals = numpy.where(norms > 0.0, normals / numpy.maximum(norms, 1e-12), [0.0, 0.0, 1.0]).astype(numpy.float32)

    # Colors: use the ones in the file, or encode the normals (or positions) as colors
    if len(ply.colors) == num_points:
      self.colors = numpy.array(ply.colors, dtype=numpy.float32)
    elif self.normals is not None:
      self.colors = 0.5 * self.normals + 0.5  # N x 3 (alpha = 1)
    else:
      self.colors = (self.vertices - min_pt) / self.extent

    # Intensities: use the ones in the file, or use the Y coordinates if they are not available or constant
    intensities = numpy.asarray(ply.intensities, dtype=numpy.float32).ravel()
    if len(intensities) != num_points or numpy.ptp(intensities) < 1e-6:
      intensities = self.vertices[:, 1]
    self.intensities = (intensities - intensities.min()) / max(float(numpy.ptp(intensities)), 1e-6)  # Normalized to [0, 1]

    # Colormap values: distance from the center (not normalized, the range is given to the shader)
    self.cmaps = numpy.linalg.norm(self.vertices - self.center, axis=1)


def load_points(path):
  ply = glk.load_ply(path)
  if ply is None or len(ply.vertices) == 0:
    return None
  return PointCloud(ply)


viewer = guik.viewer()

# The Stanford bunny in the data directory is selected by default
ply_path = glk.get_data_path() + '/models/bunny.ply'
cloud = load_points(ply_path)
if cloud is None:
  raise RuntimeError(f'failed to load {ply_path}')

# Rendering settings
color_modes = ['RAINBOW', 'FLAT_COLOR', 'VERTEX_COLOR (colors)', 'VERTEX_COLOR (intensities)', 'VERTEX_COLORMAP (cmaps)']
colormap_names = list(glk.COLORMAP.__members__.keys())
color_mode = 0
colormap = colormap_names.index('TURBO')
flat_color = numpy.array([1.0, 0.5, 0.2, 1.0], dtype=numpy.float32)
point_size = 0.05
metric = True
circle = True
y_up = True  # The bunny model is Y-up
show_normals = False


# Register the point cloud to the viewer with the current settings
def update_points():
  # Model matrix that fits the point cloud into a 10m box around the origin
  scale = 10.0 / cloud.extent
  model = numpy.identity(4, dtype=numpy.float32)
  model[:3, :3] *= scale
  model[:3, 3] = -scale * cloud.center
  if y_up:
    rot_x = numpy.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=numpy.float32)  # 90 deg around X
    model = rot_x @ model

  cmap_type = glk.COLORMAP.__members__[colormap_names[colormap]]

  # Per-point attributes are optional keyword arguments of update_points()
  attributes = {'normals': cloud.normals}  # Normals are used by screen space effects (e.g., lighting)
  if color_mode == 0:  # Color encodes the height (Z) of each point
    setting = guik.Rainbow(model).add('z_range', numpy.array([-5.0, 5.0], dtype=numpy.float32))
  elif color_mode == 1:  # Single color
    setting = guik.FlatColor(*flat_color, model)
  elif color_mode == 2:  # Per-point RGB(A) colors
    attributes['colors'] = cloud.colors
    setting = guik.VertexColor(model)
  elif color_mode == 3:  # Per-point intensities converted to RGBA colors with a colormap on CPU
    attributes['intensities'] = cloud.intensities
    attributes['intensity_colormap'] = cmap_type
    setting = guik.VertexColor(model)
  else:  # Per-point scalar values converted to colors with the viewer's colormap on GPU
    attributes['cmaps'] = cloud.cmaps
    setting = guik.VertexColorMap(model).add('cmap_range', numpy.array([0.0, 0.5 * cloud.extent], dtype=numpy.float32))

  # Point shape: size in meters (metric) or pixels (screenspace), and circle or rectangle
  setting.set_point_shape(point_size, metric, circle)

  # Colormap used for RAINBOW and VERTEX_COLORMAP
  viewer.set_colormap(cmap_type)
  viewer.update_points('points', cloud.vertices, shader_setting=setting, **attributes)

  if show_normals and cloud.normals is not None:
    viewer.update_drawable('normals', glk.PointNormalsBuffer(cloud.vertices, cloud.normals, 0.01 * cloud.extent), guik.FlatColor(0.2, 1.0, 0.2, 1.0, model))
  else:
    viewer.remove_drawable('normals')


update_points()


def points_ui():
  global cloud, ply_path, color_mode, colormap, flat_color, point_size, metric, circle, y_up, show_normals
  imgui.begin('points', True, imgui.WindowFlags_AlwaysAutoResize)

  # File IO
  imgui.text(f'{ply_path} ({len(cloud.vertices)} points)')
  changed = False
  if imgui.button('Load PLY'):
    # The file dialog starts with the current file selected
    path = pfd.open_file('Select a PLY file', ply_path)
    if path:
      loaded = load_points(path)
      if loaded is not None:
        cloud, ply_path, changed = loaded, path, True
      else:
        viewer.append_text(f'failed to load {path}')

  imgui.same_line()
  if imgui.button('Save PLY'):
    path = pfd.save_file('Select a destination path', 'points.ply')
    if path:
      # Save the points with the attributes
      ply = glk.PLYData()
      ply.vertices = cloud.vertices
      if cloud.normals is not None:
        ply.normals = cloud.normals
      ply.colors = numpy.hstack([cloud.colors[:, :3], numpy.ones((len(cloud.colors), 1), dtype=numpy.float32)])
      ply.intensities = cloud.intensities
      viewer.append_text(f'saved to {path}' if glk.save_ply(path, ply) else f'failed to save {path}')

  # Rendering settings
  imgui.separator()
  c1, color_mode = imgui.combo('color mode', color_mode, color_modes)
  c2, colormap = imgui.combo('colormap', colormap, colormap_names)
  c3, flat_color = imgui.color_edit4('flat color', flat_color)

  imgui.separator()
  c4, point_size = imgui.drag_float('point size', point_size, 0.001 if metric else 0.1, 0.001, 100.0)
  c5, metric = imgui.checkbox('metric', metric)
  if c5:
    point_size = 0.05 if metric else 5.0  # meters or pixels
  imgui.same_line()
  c6, circle = imgui.checkbox('circle', circle)

  imgui.separator()
  c7, y_up = imgui.checkbox('Y-up', y_up)
  imgui.same_line()
  c8, show_normals = imgui.checkbox('show normals', show_normals)
  imgui.end()

  if changed or any([c1, c2, c3, c4, c5, c6, c7, c8]):
    update_points()


viewer.register_ui_callback('points_ui', points_ui)

viewer.spin()
