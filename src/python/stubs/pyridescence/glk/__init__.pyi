"""
"""
from __future__ import annotations
import numpy
import pyridescence.guik
import typing
import typing_extensions
from . import primitives
__all__: list[str] = ['AUTUMN', 'BLUE_RED', 'CIVIDIS', 'COLORMAP', 'COOL_WARM', 'CURL', 'Drawable', 'DrawableContainer', 'ElevationMap', 'ElevationMapBuilder', 'GLSLShader', 'GREAN_YELLOW', 'GridMap', 'HELIX', 'IndexedPointCloudBuffer', 'JET', 'Lines', 'MeshRenderingOptions', 'NaiveScreenSpaceAmbientOcclusion', 'OCEAN', 'PASTEL', 'PHASE', 'PLYData', 'PUBUGN', 'PointCloudBuffer', 'SPRING', 'SUMMER', 'ScreenEffect', 'ScreenSpaceAmbientOcclusion', 'ScreenSpaceLighting', 'TURBID', 'TURBO', 'Texture', 'ThinLines', 'VEGETATION','VoxelMap', 'WINTER', 'colormap', 'colormap_categorical', 'colormap_categoricalf', 'colormapf', 'create_pointcloud_buffer', 'create_texture', 'create_texture_f', 'get_data_path', 'load_ply', 'primitives', 'save_ply', 'set_data_path']
class COLORMAP:
    """
    Members:
    
      TURBO
    
      JET
    
      CIVIDIS
    
      OCEAN
    
      SPRING
    
      SUMMER
    
      AUTUMN
    
      WINTER
    
      GREAN_YELLOW
    
      BLUE_RED
    
      PUBUGN
    
      TURBID
    
      PASTEL
    
      HELIX
    
      PHASE
    
      VEGETATION
    
      CURL
    
      COOL_WARM
    """
    AUTUMN: typing.ClassVar[COLORMAP]  # value = <COLORMAP.AUTUMN: 6>
    BLUE_RED: typing.ClassVar[COLORMAP]  # value = <COLORMAP.BLUE_RED: 9>
    CIVIDIS: typing.ClassVar[COLORMAP]  # value = <COLORMAP.CIVIDIS: 2>
    COOL_WARM: typing.ClassVar[COLORMAP]  # value = <COLORMAP.COOL_WARM: 17>
    CURL: typing.ClassVar[COLORMAP]  # value = <COLORMAP.CURL: 16>
    GREAN_YELLOW: typing.ClassVar[COLORMAP]  # value = <COLORMAP.GREAN_YELLOW: 8>
    HELIX: typing.ClassVar[COLORMAP]  # value = <COLORMAP.HELIX: 13>
    JET: typing.ClassVar[COLORMAP]  # value = <COLORMAP.JET: 1>
    OCEAN: typing.ClassVar[COLORMAP]  # value = <COLORMAP.OCEAN: 3>
    PASTEL: typing.ClassVar[COLORMAP]  # value = <COLORMAP.PASTEL: 12>
    PHASE: typing.ClassVar[COLORMAP]  # value = <COLORMAP.PHASE: 14>
    PUBUGN: typing.ClassVar[COLORMAP]  # value = <COLORMAP.PUBUGN: 10>
    SPRING: typing.ClassVar[COLORMAP]  # value = <COLORMAP.SPRING: 4>
    SUMMER: typing.ClassVar[COLORMAP]  # value = <COLORMAP.SUMMER: 5>
    TURBID: typing.ClassVar[COLORMAP]  # value = <COLORMAP.TURBID: 11>
    TURBO: typing.ClassVar[COLORMAP]  # value = <COLORMAP.TURBO: 0>
    VEGETATION: typing.ClassVar[COLORMAP]  # value = <COLORMAP.VEGETATION: 15>
    WINTER: typing.ClassVar[COLORMAP]  # value = <COLORMAP.WINTER: 7>
    __members__: typing.ClassVar[dict[str, COLORMAP]]  # value = {'TURBO': <COLORMAP.TURBO: 0>, 'JET': <COLORMAP.JET: 1>, 'CIVIDIS': <COLORMAP.CIVIDIS: 2>, 'OCEAN': <COLORMAP.OCEAN: 3>, 'SPRING': <COLORMAP.SPRING: 4>, 'SUMMER': <COLORMAP.SUMMER: 5>, 'AUTUMN': <COLORMAP.AUTUMN: 6>, 'WINTER': <COLORMAP.WINTER: 7>, 'GREAN_YELLOW': <COLORMAP.GREAN_YELLOW: 8>, 'BLUE_RED': <COLORMAP.BLUE_RED: 9>, 'PUBUGN': <COLORMAP.PUBUGN: 10>, 'TURBID': <COLORMAP.TURBID: 11>, 'PASTEL': <COLORMAP.PASTEL: 12>, 'HELIX': <COLORMAP.HELIX: 13>, 'PHASE': <COLORMAP.PHASE: 14>, 'VEGETATION': <COLORMAP.VEGETATION: 15>, 'CURL': <COLORMAP.CURL: 16>, 'COOL_WARM': <COLORMAP.COOL_WARM: 17>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Drawable:
    def __init__(self) -> None:
        ...
    def draw(self, shader: GLSLShader) -> None:
        ...
class DrawableContainer(Drawable):
    """
    Container to store multiple drawables and draw them in order.
    It can also store a shader setting for each drawable.
    
    Shader settings stored in the container are applied right before drawing the corresponding drawable and are not restored afterward,
    thus they also affect the subsequent drawables in the container that do not have their own shader settings.
    """
    skip_model_matrix: bool
    """
    If True, the model matrices of the shader settings stored in the container are not applied.
    """
    def __len__(self) -> int:
        ...
    @typing.overload
    def __init__(self, skip_model_matrix_setting: bool = True) -> None:
        """
        Create an empty container.
        
        Args:
            skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
        """
    @typing.overload
    def __init__(self, drawables: list[Drawable], skip_model_matrix_setting: bool = True) -> None:
        """
        Create a container with drawables.
        
        Args:
            drawables: List of drawables to be added to the container.
            skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
        """
    def clear(self) -> None:
        """
        Clear all drawables in the container.
        """
    @typing.overload
    def push_back(self, drawable: Drawable) -> None:
        """
        Add a drawable to the container.
        
        Args:
            drawable: Drawable to be added.
        """
    @typing.overload
    def push_back(self, drawable: Drawable, shader_setting: pyridescence.guik.ShaderSetting) -> None:
        """
        Add a drawable to the container with a shader setting.
        
        The shader setting is copied, and later changes to it are not reflected.
        
        Args:
            drawable: Drawable to be added.
            shader_setting: Shader setting to be used when drawing the drawable.
        """
    def size(self) -> int:
        """
        Number of drawables in the container.
        """
class GLSLShader:
    def attrib(self, name: str) -> int:
        ...
    def get_uniform4f(self, name: str) -> numpy.ndarray[numpy.float32[4, 1]]:
        ...
    def get_uniform_matrix4f(self, name: str) -> numpy.ndarray[numpy.float32[4, 4]]:
        ...
    def get_uniformf(self, name: str) -> float:
        ...
    def get_uniformi(self, name: str) -> int:
        ...
    def id(self) -> int:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: int) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: float) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.float32[2, 1]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.float32[3, 1]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.float32[4, 1]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.float32[4, 4]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.int32[2, 1]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.int32[3, 1]]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: numpy.ndarray[numpy.int32[4, 1]]) -> None:
        ...
    def uniform(self, name: str) -> int:
        ...
    def unuse(self) -> None:
        ...
    def use(self) -> None:
        ...
class ElevationMap(Drawable):
    def __init__(self, builder: ElevationMapBuilder, options: MeshRenderingOptions = ...) -> None:
        ...
class ElevationMapBuilder:
    def __init__(self, resolution: float, size: numpy.ndarray[numpy.int32[2, 1]], origin: numpy.ndarray[numpy.float32[2, 1]]) -> None:
        ...
    def calc_cell_indices(self, cells: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.uint64]:
        ...
    def calc_cmap_gradient(self, window_size: int = 3) -> None:
        ...
    def filter_elevation_map(self, process_valid_cells: bool = True, window_size: int = 3) -> None:
        ...
    def set_cmap(self, cells: numpy.ndarray[numpy.float32], cmap_index: int, cell_indices: numpy.ndarray[numpy.uint64], average: bool = False) -> None:
        ...
    def set_elevation(self, cells: numpy.ndarray[numpy.float32], elevation_index: int, cell_indices: numpy.ndarray[numpy.uint64], average: bool = False) -> None:
        ...
    @property
    def color_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
        ...
    @color_map.setter
    def color_map(self, arg1: numpy.ndarray[numpy.float32[m, n]]) -> None:
        ...
    @property
    def count_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
        ...
    @property
    def elevation_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
        ...
    @elevation_map.setter
    def elevation_map(self, arg1: numpy.ndarray[numpy.float32[m, n]]) -> None:
        ...
    @property
    def origin(self) -> numpy.ndarray[numpy.float32[2, 1]]:
        ...
    @property
    def resolution(self) -> float:
        ...
    @property
    def size(self) -> numpy.ndarray[numpy.int32[2, 1]]:
        ...
class GridMap(Drawable):
    """
    Drawable that represents a 2D grid map with color values.
    
    values is a (height, width) array ((height, width, 4) for RGBA mode), and values[y, x] is the value of cell (x, y).
    The first row (values[0, :]) lies at -y.
    The grid map lies on the XY plane, and the center is at the origin (0, 0, 0),
    thus it extends from (-resolution * width / 2, -resolution * height / 2) to (resolution * width / 2, resolution * height / 2).
    Use guik.TextureColor as shader setting.
    
    uint8 arrays use the uint8 interpretation of ColorMode, and arrays of other dtypes are converted to float32 and use the float interpretation.
    """
    class ColorMode:
        """
        Defines how the color values are interpreted.
        
        For uint8 values:
          RAW: rgb = values[y, x]
          TURBO: rgb = colormap(TURBO, values[y, x])
          PROB: rgb = 100 - values[y, x]
          PROB_TURBO: rgb = colormap(TURBO, (100 - values[y, x]) * 255.0 / 100.0)
          RGBA: rgba = values[y, x, :]
        For float values:
          RAW: rgb = scale * values[y, x]
          TURBO: rgb = colormapf(TURBO, scale * values[y, x])
          PROB: rgb = 1.0 - scale * values[y, x]
          PROB_TURBO: rgb = colormapf(TURBO, 1.0 - scale * values[y, x])
          RGBA: rgba = values[y, x, :] (scale is not applied)
        
        
        Members:
        
          RAW
        
          TURBO
        
          PROB
        
          PROB_TURBO
        
          RGBA
        """
        PROB: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.PROB: 2>
        PROB_TURBO: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.PROB_TURBO: 3>
        RAW: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.RAW: 0>
        RGBA: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.RGBA: 4>
        TURBO: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.TURBO: 1>
        __members__: typing.ClassVar[dict[str, GridMap.ColorMode]]  # value = {'RAW': <ColorMode.RAW: 0>, 'TURBO': <ColorMode.TURBO: 1>, 'PROB': <ColorMode.PROB: 2>, 'PROB_TURBO': <ColorMode.PROB_TURBO: 3>, 'RGBA': <ColorMode.RGBA: 4>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    PROB: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.PROB: 2>
    PROB_TURBO: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.PROB_TURBO: 3>
    RAW: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.RAW: 0>
    RGBA: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.RGBA: 4>
    TURBO: typing.ClassVar[GridMap.ColorMode]  # value = <ColorMode.TURBO: 1>
    @typing.overload
    def __init__(self, resolution: float, values: numpy.ndarray[numpy.float32], alpha: float = 1.0, mode: GridMap.ColorMode = GridMap.ColorMode.PROB, scale: float = 1.0) -> None:
        """
        Create a grid map from float values.
        
        Args:
            resolution: Resolution of each grid cell in world units.
            values: Color values of shape (height, width), or (height, width, 4) for RGBA mode. Converted to float32.
            alpha: Alpha value for the entire map. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
            scale: Scale factor for the color values. Ignored if mode is RGBA.
        """
    @typing.overload
    def __init__(self, resolution: float, values: numpy.ndarray[numpy.uint8], alpha: int = 255, mode: GridMap.ColorMode = GridMap.ColorMode.PROB) -> None:
        """
        Create a grid map from uint8 values.
        
        Args:
            resolution: Resolution of each grid cell in world units.
            values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
            alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
        """
    @typing.overload
    def update_color(self, values: numpy.ndarray[numpy.float32], alpha: float = 1.0, mode: GridMap.ColorMode = GridMap.ColorMode.PROB, scale: float = 1.0) -> None:
        """
        Update the colors with float values.
        
        The map extent given at construction does not change. If the shape of values differs from the original one, the colors are stretched to fit the extent.
        
        Args:
            values: Color values of shape (height, width), or (height, width, 4) for RGBA mode. Converted to float32.
            alpha: Alpha value for the entire map. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
            scale: Scale factor for the color values. Ignored if mode is RGBA.
        """
    @typing.overload
    def update_color(self, values: numpy.ndarray[numpy.uint8], alpha: int = 255, mode: GridMap.ColorMode = GridMap.ColorMode.PROB) -> None:
        """
        Update the colors with uint8 values.
        
        The map extent given at construction does not change. If the shape of values differs from the original one, the colors are stretched to fit the extent.
        
        Args:
            values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
            alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
        """
class IndexedPointCloudBuffer(Drawable):
    """
    Drawable that represents a point cloud with an index buffer.
    It is useful for rendering a subset of points without duplicating the point cloud data.
    """
    def __init__(self, cloud_buffer: PointCloudBuffer, indices: numpy.ndarray[numpy.int64] | typing.Sequence[int]) -> None:
        """
        Create an indexed point cloud buffer.
        
        Args:
            cloud_buffer: Point cloud buffer that holds the point data. It is shared, not copied.
            indices: 1-dimensional integer array of point indices to be rendered. Each index must be in [0, number of points in cloud_buffer).
        """
    def __len__(self) -> int:
        ...
    def ebo_id(self) -> int:
        """
        OpenGL ID of the element (index) buffer.
        """
    def size(self) -> int:
        """
        Number of indices.
        """
class Lines(Drawable):
    @typing.overload
    def __init__(self, thickness: float, vertices: list[numpy.ndarray[numpy.float32[3, 1]]], colors: list[numpy.ndarray[numpy.float32[4, 1]]]) -> None:
        ...
    @typing.overload
    def __init__(self, thickness: float, vertices: numpy.ndarray[numpy.float32], colors: numpy.ndarray[numpy.float32]) -> None:
        ...
    @typing.overload
    def __init__(self, x: numpy.ndarray[numpy.float32[m, 1]] = ..., y: numpy.ndarray[numpy.float32[m, 1]] = ..., z: numpy.ndarray[numpy.float32[m, 1]] = ..., c: numpy.ndarray[numpy.float32[m, 1]] = ..., width: float = 0.10000000149011612, line_strip: bool = True) -> None:
        ...
class MeshRenderingOptions:
    draw_edges: bool
    draw_faces: bool
    edge_line_width: float
    def __init__(self) -> None:
        ...
    def set_edge_alpha(self, alpha: float) -> MeshRenderingOptions:
        ...
    def set_edge_color(self, color: numpy.ndarray[numpy.float32[4, 1]]) -> MeshRenderingOptions:
        ...
    def set_face_alpha(self, alpha: float) -> MeshRenderingOptions:
        ...
    def set_face_color(self, color: numpy.ndarray[numpy.float32[4, 1]]) -> MeshRenderingOptions:
        ...
class NaiveScreenSpaceAmbientOcclusion(ScreenEffect):
    def __init__(self) -> None:
        ...
class PLYData:
    colors: numpy.ndarray[numpy.float32[m, n]]
    comments: list[str]
    indices: numpy.ndarray[numpy.int32[m, 1]]
    intensities: numpy.ndarray[numpy.float32[m, 1]]
    normals: numpy.ndarray[numpy.float32[m, n]]
    vertices: numpy.ndarray[numpy.float32[m, n]]
    def __init__(self) -> None:
        ...
class PointCloudBuffer(Drawable):
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32]) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    def add_buffer(self, attribute_name: str, data: numpy.ndarray[numpy.float32[m, n]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: list[numpy.ndarray[numpy.float32[4, 1]]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: numpy.ndarray[numpy.float32]) -> None:
        ...
    def add_colormap(self, cmap: list[float], scale: float = 1.0) -> None:
        ...
    def add_intensity(self, colormap: COLORMAP, intensities: list[float], scale: float = 1.0) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: numpy.ndarray[numpy.float32]) -> None:
        ...
    def set_colormap_buffer(self, colormap_buffer: str) -> None:
        ...
class PointNormalsBuffer(Drawable):
    """
    Drawable that visualizes point normals as line segments.
    Only the normal lines (from p to p + n * normal_length) are drawn. Points themselves are not rendered.
    """
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32], normals: numpy.ndarray[numpy.float32], normal_length: float = 1.0) -> None:
        """
        Create a point normals buffer.
        
        Args:
            points: Point positions (N x 3 or N x 4 array). The 4th column is ignored.
            normals: Normal vectors (N x 3 or N x 4 array). The 4th column is ignored.
            normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
        """
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]], normals: list[numpy.ndarray[numpy.float32[3, 1]]], normal_length: float = 1.0) -> None:
        """
        Create a point normals buffer.
        
        Args:
            points: List of point positions.
            normals: List of normal vectors. Must have the same size as points.
            normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
        """
class ScreenEffect:
    pass
class ScreenSpaceAmbientOcclusion(ScreenEffect):
    def __init__(self) -> None:
        ...
class ScreenSpaceLighting(ScreenEffect):
    class DIFFUSE_MODEL:
        """
        Members:
        
          ZERO
        
          LAMBERT
        
          DISNEY
        
          NORMALIZED_DISNEY
        
          OREN_NAYAR
        """
        DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.DISNEY: 3>
        LAMBERT: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.LAMBERT: 2>
        NORMALIZED_DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.NORMALIZED_DISNEY: 4>
        OREN_NAYAR: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.OREN_NAYAR: 5>
        ZERO: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.ZERO: 0>
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.DIFFUSE_MODEL]]  # value = {'ZERO': <DIFFUSE_MODEL.ZERO: 0>, 'LAMBERT': <DIFFUSE_MODEL.LAMBERT: 2>, 'DISNEY': <DIFFUSE_MODEL.DISNEY: 3>, 'NORMALIZED_DISNEY': <DIFFUSE_MODEL.NORMALIZED_DISNEY: 4>, 'OREN_NAYAR': <DIFFUSE_MODEL.OREN_NAYAR: 5>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class VoxelMapOptions:
        def __init__(self):
            ...
        def set_voxel_alpha(self, alpha: float):
            ...
        def set_voxel_color(self, color: numpy.ndarray[np.int32[4,1]]):
            ...
        def set_edge_alpha(self, alpha: float):
            ...
        def set_edge_color(self, color: numpy.ndarray[np.int32[4,1]]):
            ...
    class VoxelMap(Drawable):
        def __init__(voxels: numpy.ndarray[np.int32[m, 3]], int num_voxels, voxel_resolution: float, voxel_options:VoxelMapOptions = VoxelMapOptions()):
            ...
    class IRIDESCENCE_MODEL:
        """
        Members:
        
          ZERO
        
          IRIDESCENCE1
        
          IRIDESCENCE2
        
          IRIDESCENCE3
        """
        IRIDESCENCE1: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE1: 1>
        IRIDESCENCE2: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE2: 2>
        IRIDESCENCE3: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE3: 3>
        ZERO: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.ZERO: 0>
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.IRIDESCENCE_MODEL]]  # value = {'ZERO': <IRIDESCENCE_MODEL.ZERO: 0>, 'IRIDESCENCE1': <IRIDESCENCE_MODEL.IRIDESCENCE1: 1>, 'IRIDESCENCE2': <IRIDESCENCE_MODEL.IRIDESCENCE2: 2>, 'IRIDESCENCE3': <IRIDESCENCE_MODEL.IRIDESCENCE3: 3>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class OCCLUSION_MODEL:
        """
        Members:
        
          ZERO
        
          AMBIENT_OCCLUSION
        """
        AMBIENT_OCCLUSION: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]  # value = <OCCLUSION_MODEL.AMBIENT_OCCLUSION: 1>
        ZERO: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]  # value = <OCCLUSION_MODEL.ZERO: 0>
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.OCCLUSION_MODEL]]  # value = {'ZERO': <OCCLUSION_MODEL.ZERO: 0>, 'AMBIENT_OCCLUSION': <OCCLUSION_MODEL.AMBIENT_OCCLUSION: 1>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class SPECULAR_MODEL:
        """
        Members:
        
          ZERO
        
          PHONG
        
          BLINN_PHONG
        
          COOK_TORRANCE
        """
        BLINN_PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.BLINN_PHONG: 2>
        COOK_TORRANCE: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.COOK_TORRANCE: 3>
        PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.PHONG: 1>
        ZERO: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.ZERO: 0>
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.SPECULAR_MODEL]]  # value = {'ZERO': <SPECULAR_MODEL.ZERO: 0>, 'PHONG': <SPECULAR_MODEL.PHONG: 1>, 'BLINN_PHONG': <SPECULAR_MODEL.BLINN_PHONG: 2>, 'COOK_TORRANCE': <SPECULAR_MODEL.COOK_TORRANCE: 3>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    AMBIENT_OCCLUSION: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]  # value = <OCCLUSION_MODEL.AMBIENT_OCCLUSION: 1>
    BLINN_PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.BLINN_PHONG: 2>
    COOK_TORRANCE: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.COOK_TORRANCE: 3>
    DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.DISNEY: 3>
    IRIDESCENCE1: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE1: 1>
    IRIDESCENCE2: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE2: 2>
    IRIDESCENCE3: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.IRIDESCENCE3: 3>
    LAMBERT: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.LAMBERT: 2>
    NORMALIZED_DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.NORMALIZED_DISNEY: 4>
    OREN_NAYAR: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]  # value = <DIFFUSE_MODEL.OREN_NAYAR: 5>
    PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]  # value = <SPECULAR_MODEL.PHONG: 1>
    ZERO: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]  # value = <IRIDESCENCE_MODEL.ZERO: 0>
    def __init__(self) -> None:
        ...
    def num_lights(self) -> int:
        ...
    def set_albedo(self, arg0: float) -> None:
        ...
    def set_diffuse_model(self, arg0: ScreenSpaceLighting.DIFFUSE_MODEL) -> None:
        ...
    def set_iridescence_model(self, arg0: ScreenSpaceLighting.IRIDESCENCE_MODEL) -> None:
        ...
    def set_light(self, arg0: int, arg1: numpy.ndarray[numpy.float32[3, 1]], arg2: numpy.ndarray[numpy.float32[4, 1]]) -> None:
        ...
    def set_occlusion_model(self, arg0: ScreenSpaceLighting.OCCLUSION_MODEL) -> None:
        ...
    def set_roughness(self, arg0: float) -> None:
        ...
    def set_specular_model(self, arg0: ScreenSpaceLighting.SPECULAR_MODEL) -> None:
        ...
class Texture:
    """
    2D OpenGL texture.
    All methods must be called in the thread that owns the OpenGL context.
    Pixel data are assumed to be tightly packed (no row padding).
    """
    @typing.overload
    def __init__(self, size: numpy.ndarray[numpy.int32[2, 1]], internal_format: int, format: int, type: int, bytes: typing_extensions.Buffer) -> None:
        """
        Create a texture from raw pixel data.
        
        Args:
            size: Texture size (width, height).
            internal_format: Internal format of the texture (e.g., gl.RGBA8, gl.RGBA32F).
            format: Format of the pixel data (e.g., gl.RGBA, gl.RGB, gl.RED).
            type: Data type of the pixel data (e.g., gl.UNSIGNED_BYTE, gl.FLOAT).
            bytes: Pixel data as an object supporting the buffer protocol (e.g., numpy array, bytes, bytearray).
                   The first row is uploaded as the first (v=0) row of the texture. If empty, the texture is created but not initialized.
        """
    @typing.overload
    def __init__(self, size: numpy.ndarray[numpy.int32[2, 1]], internal_format: int, format: int, type: int, bytes: list[int]) -> None:
        """
        Create a texture from raw pixel data given as a list of bytes.
        
        Args:
            size: Texture size (width, height).
            internal_format: Internal format of the texture (e.g., gl.RGBA8, gl.RGBA32F).
            format: Format of the pixel data (e.g., gl.RGBA, gl.RGB, gl.RED).
            type: Data type of the pixel data (e.g., gl.UNSIGNED_BYTE, gl.FLOAT).
            bytes: Pixel data as a list of byte values. If empty, the texture is created but not initialized.
        """
    @typing.overload
    def bind(self) -> None:
        """
        Bind the texture to GL_TEXTURE_2D of the active texture unit.
        """
    @typing.overload
    def bind(self, unit: int) -> None:
        """
        Bind the texture to the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.
        """
    def id(self) -> int:
        """
        OpenGL texture ID. Note that it changes when set_size() is called.
        """
    def read_pixels(self, format: int = 6408, type: int = 5121, num_components: int = -1) -> numpy.ndarray:
        """
        Read the pixel data (mip level 0) from the texture.
        
        Args:
            format: Format of the pixel data to read (e.g., gl.RGBA, gl.RGB, gl.RED, gl.DEPTH_COMPONENT, gl.RGBA_INTEGER).
            type: Data type of the pixel data to read. It determines the dtype of the returned array
                  (UNSIGNED_BYTE: uint8, BYTE: int8, UNSIGNED_SHORT: uint16, SHORT: int16, HALF_FLOAT: float16,
                   UNSIGNED_INT: uint32, INT: int32, FLOAT: float32).
            num_components: Number of components per pixel. If <= 0, it is derived from format.
        
        Returns:
            Array of shape (height, width, num_components), or (height, width) if num_components is 1.
            The first row is the first (v=0) row of the texture. For the viewer's frame buffers, it is the bottom row of the screen.
        """
    def set_clamp_mode(self, mode: int) -> None:
        """
        Set the wrap mode for both S and T (e.g., gl.CLAMP_TO_EDGE, gl.REPEAT).
        """
    def set_filter_mode(self, mode: int) -> None:
        """
        Set the min/mag filter mode (gl.LINEAR or gl.NEAREST).
        """
    def set_size(self, size: numpy.ndarray[numpy.int32[2, 1]]) -> None:
        """
        Change the size of the texture.
        The texture is re-created: the texture data are discarded, the texture ID changes, and the filter and clamp modes are reset to the defaults.
        Do not call this on the viewer's frame buffer textures (e.g., color_buffer()).
        """
    def size(self) -> numpy.ndarray[numpy.int32[2, 1]]:
        """
        Texture size (width, height).
        """
    @typing.overload
    def unbind(self) -> None:
        """
        Unbind GL_TEXTURE_2D of the active texture unit.
        """
    @typing.overload
    def unbind(self, unit: int) -> None:
        """
        Unbind GL_TEXTURE_2D of the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.
        """
class ThinLines(Drawable):
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32], line_strip: bool = False) -> None:
        ...
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32], colors: numpy.ndarray[numpy.float32], line_strip: bool = False) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]], line_strip: bool = False) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]], colors: list[numpy.ndarray[numpy.float32[4, 1]]], line_strip: bool = False) -> None:
        ...
    @typing.overload
    def __init__(self, x: numpy.ndarray[numpy.float32[m, 1]] = ..., y: numpy.ndarray[numpy.float32[m, 1]] = ..., z: numpy.ndarray[numpy.float32[m, 1]] = ..., c: numpy.ndarray[numpy.float32[m, 1]] = ..., line_strip: bool = True) -> None:
        ...
    def set_line_width(self, width: float) -> None:
        ...
def colormap(colormap_type: COLORMAP, x: int) -> numpy.ndarray[numpy.int32[4, 1]]:
    ...
def colormap_categorical(colormap_type: COLORMAP, x: int, num_categories: int) -> numpy.ndarray[numpy.int32[4, 1]]:
    ...
def colormap_categoricalf(colormap_type: COLORMAP, x: int, num_categories: int) -> numpy.ndarray[numpy.float32[4, 1]]:
    ...
def colormapf(colormap_type: COLORMAP, x: float) -> numpy.ndarray[numpy.float32[4, 1]]:
    ...
def create_pointcloud_buffer(points: numpy.ndarray[numpy.float32[m, 3]], colors: numpy.ndarray[numpy.float32[m, 4]] = ...) -> PointCloudBuffer:
    ...
def create_texture(image_uint8: numpy.ndarray[numpy.uint8]) -> Texture:
    ...
def create_texture_f(image_float: numpy.ndarray[numpy.float32]) -> Texture:
    ...
def get_data_path() -> str:
    ...
def load_ply(filename: str) -> PLYData:
    ...
@typing.overload
def save_ply(filename: str, ply: PLYData, binary: bool = True) -> bool:
    ...
@typing.overload
def save_ply(arg0: str, arg1: numpy.ndarray[numpy.float32[m, n]]) -> bool:
    ...
def set_data_path(arg0: str) -> None:
    ...
AUTUMN: COLORMAP  # value = <COLORMAP.AUTUMN: 6>
BLUE_RED: COLORMAP  # value = <COLORMAP.BLUE_RED: 9>
CIVIDIS: COLORMAP  # value = <COLORMAP.CIVIDIS: 2>
COOL_WARM: COLORMAP  # value = <COLORMAP.COOL_WARM: 17>
CURL: COLORMAP  # value = <COLORMAP.CURL: 16>
GREAN_YELLOW: COLORMAP  # value = <COLORMAP.GREAN_YELLOW: 8>
HELIX: COLORMAP  # value = <COLORMAP.HELIX: 13>
JET: COLORMAP  # value = <COLORMAP.JET: 1>
OCEAN: COLORMAP  # value = <COLORMAP.OCEAN: 3>
PASTEL: COLORMAP  # value = <COLORMAP.PASTEL: 12>
PHASE: COLORMAP  # value = <COLORMAP.PHASE: 14>
PUBUGN: COLORMAP  # value = <COLORMAP.PUBUGN: 10>
SPRING: COLORMAP  # value = <COLORMAP.SPRING: 4>
SUMMER: COLORMAP  # value = <COLORMAP.SUMMER: 5>
TURBID: COLORMAP  # value = <COLORMAP.TURBID: 11>
TURBO: COLORMAP  # value = <COLORMAP.TURBO: 0>
VEGETATION: COLORMAP  # value = <COLORMAP.VEGETATION: 15>
WINTER: COLORMAP  # value = <COLORMAP.WINTER: 7>
