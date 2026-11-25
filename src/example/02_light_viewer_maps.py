#!/usr/bin/python3
import numpy
from pyridescence import *

viewer = guik.LightViewer.instance()
viewer.disable_xy_grid()

hover = guik.HoveredDrawings()
viewer.register_ui_callback('hovered', hover.create_callback())

resolution = 0.1
width_m = 5.0
height_m = 5.0
width = int(width_m / resolution)
height = int(height_m / resolution)

# Surface points on a (height, width) grid. surface_points[y * width + x] is the point of cell (x, y).
xs, ys = numpy.meshgrid(numpy.arange(width) * resolution, numpy.arange(height) * resolution)
elevation = (0.25 * numpy.sin(xs * 3.0) * numpy.cos(ys * 3.0)).astype(numpy.float32)  # (height, width)
surface_points = numpy.stack([xs, ys, elevation], axis=-1).reshape(-1, 3).astype(numpy.float32)

# glk.GridMap
gridmap = glk.GridMap(resolution, elevation, alpha=1.0, mode=glk.GridMap.ColorMode.TURBO, scale=4.0)
viewer.update_drawable('gridmap', gridmap, guik.TextureColor().translate(-5.0, -5.0, 0.0))
viewer.update_coord('gridmap_coord', guik.VertexColor().translate(-5.0, -5.0, 0.0))
hover.add_text_on('gridmap', '2D grid map')

# glk.ElevationMap (from 3D surface points)
# Rainbow().set_color_mode(4) is equivalent to guik::VertexColorMap in C++ (color from the vertex colormap attribute)
elev_map_builder = glk.ElevationMapBuilder(resolution, (width, height), (0.0, 0.0))
cell_indices = elev_map_builder.calc_cell_indices(surface_points)
elev_map_builder.set_elevation(surface_points, 2, cell_indices)
elev_map_builder.set_cmap(surface_points, 2, cell_indices)

elev_map = glk.ElevationMap(elev_map_builder, glk.MeshRenderingOptions())
viewer.update_drawable('elevation_map', elev_map, guik.Rainbow().set_color_mode(4).translate(0.0, -5.0 - width_m / 2.0, 0.0))
viewer.update_coord('elevation_map_coord', guik.VertexColor().translate(0.0, -5.0 - width_m / 2.0, 0.0))
hover.add_text_on('elevation_map', '3D elevation map')

# glk.ElevationMap (from 2D elevation map)
elev_map2_builder = glk.ElevationMapBuilder(resolution, (width, height), (0.0, 0.0))
elev_map2_builder.elevation_map = elevation
elev_map2_builder.color_map = 4.0 * elevation

elev_map2 = glk.ElevationMap(elev_map2_builder, glk.MeshRenderingOptions().set_edge_color([0.0, 0.0, 1.0, 0.2]))
viewer.update_drawable('elevation_map2', elev_map2, guik.Rainbow().set_color_mode(4).translate(0.0, 0.0, 0.0))
viewer.update_coord('elevation_map2_coord', guik.VertexColor().translate(0.0, 0.0, 0.0))
hover.add_text_on('elevation_map2', '3D elevation map')

# glk.VoxelMap
voxel_coords = numpy.floor(surface_points / resolution).astype(numpy.int32)
voxelmap = glk.VoxelMap(voxel_coords, resolution, glk.MeshRenderingOptions().set_edge_color([0.0, 0.0, 1.0, 1.0]))
viewer.update_drawable('voxelmap', voxelmap, guik.Rainbow().translate(7.5, -5.0 - width_m / 2, 0.0))
viewer.update_coord('voxelmap_coord', guik.VertexColor().translate(7.5, -5.0 - width_m / 2, 0.0))
hover.add_text_on('voxelmap', '3D voxel map')

viewer.spin()
