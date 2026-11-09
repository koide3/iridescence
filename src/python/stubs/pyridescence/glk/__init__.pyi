"""
"""
from __future__ import annotations
import numpy
import pyridescence.guik
import typing
from . import primitives
__all__: list[str] = ['AUTUMN', 'BLUE_RED', 'CHAR', 'CIVIDIS', 'COLORMAP', 'COOL_WARM', 'CURL', 'DOUBLE', 'Drawable', 'DrawableContainer', 'ElevationMap', 'ElevationMapBuilder', 'FLOAT', 'GLSLShader', 'GREAN_YELLOW', 'GridMap', 'HELIX', 'INT', 'IndexedPointCloudBuffer', 'JET', 'Lines', 'MeshRenderingOptions', 'NaiveScreenSpaceAmbientOcclusion', 'OCEAN', 'PASTEL', 'PHASE', 'PLYData', 'PLYGenericPropertyBuffer', 'PLYPropertyType', 'PUBUGN', 'PointCloudBuffer', 'PointNormalsBuffer', 'SHORT', 'SPRING', 'SUMMER', 'ScreenEffect', 'ScreenSpaceAmbientOcclusion', 'ScreenSpaceLighting', 'TURBID', 'TURBO', 'Texture', 'ThinLines', 'UCHAR', 'UINT', 'USHORT', 'VEGETATION', 'VoxelMap', 'VoxelMapOptions', 'WINTER', 'colormap', 'colormap_categorical', 'colormap_categoricalf', 'colormapf', 'create_pointcloud_buffer', 'create_texture', 'create_texture_f', 'get_data_path', 'load_ply', 'primitives', 'save_ply', 'set_data_path']
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
    @typing.overload
    def __init__(self, skip_model_matrix_setting: bool = True) -> None:
        """
        Create an empty container.
        
        Args:
            skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
        """
    @typing.overload
    def __init__(self, drawables: list[typing.Any], skip_model_matrix_setting: bool = True) -> None:
        """
        Create a container with drawables.
        
        Args:
            drawables: List of drawables to be added to the container.
            skip_model_matrix_setting: If True, the model matrices of the shader settings stored in the container are not applied.
        """
    def __len__(self) -> int:
        ...
    def clear(self) -> None:
        """
        Clear all drawables in the container.
        """
    @typing.overload
    def push_back(self, drawable: typing.Any) -> None:
        """
        Add a drawable to the container.
        
        Args:
            drawable: Drawable to be added.
        """
    @typing.overload
    def push_back(self, drawable: typing.Any, shader_setting: pyridescence.guik.ShaderSetting) -> None:
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
    @property
    def skip_model_matrix(self) -> bool:
        """
        If True, the model matrices of the shader settings stored in the container are not applied.
        """
    @skip_model_matrix.setter
    def skip_model_matrix(self, arg0: bool) -> None:
        ...
class ElevationMap(Drawable):
    """
    Elevation map drawable.
    """
    def __init__(self, builder: ElevationMapBuilder, options: MeshRenderingOptions = ...) -> None:
        """
        Create an elevation map.
        
        Args:
            builder: Builder containing the elevation map data.
            options: Rendering options.
        """
class ElevationMapBuilder:
    """
    Builder for an elevation map.
    """
    color_map: numpy.ndarray[numpy.float32[m, n]]
    elevation_map: numpy.ndarray[numpy.float32[m, n]]
    def __init__(self, resolution: float, size: numpy.ndarray[numpy.int32[2, 1]], origin: numpy.ndarray[numpy.float32[2, 1]]) -> None:
        """
        Create an elevation map builder.
        
        Args:
            resolution: Resolution of the map (size of each cell).
            size: Size of the map in number of cells (width, height).
            origin: Top-left corner of the map in world coordinates.
        """
    def calc_cell_indices(self, cells: numpy.ndarray[numpy.float32]) -> numpy.ndarray[numpy.uint64]:
        """
        Calculate the cell indices for a given set of cells.
        
        Args:
            cells: Cells for which to calculate the indices (N x M array, M >= 2). cells[i, 0] and cells[i, 1] are the x and y coordinates of the cell in world coordinates.
        
        Returns:
            Indices of the cells in the elevation map.
        """
    def calc_cmap_gradient(self, window_size: int = 3) -> None:
        """
        Replace the color map with its gradient magnitude.
        
        Args:
            window_size: Size of the filter window (3, 5, or 7).
        """
    def filter_elevation_map(self, process_valid_cells: bool = True, window_size: int = 3) -> None:
        """
        Apply local averaging filter to the elevation (and color) map.
        
        Args:
            process_valid_cells: If True, all cells are processed (smoothed). If False, only invalid cells (NaN) are processed.
            window_size: Size of the filter window (3, 5, or 7).
        """
    def set_cmap(self, cells: numpy.ndarray[numpy.float32], cmap_index: int, cell_indices: numpy.ndarray[numpy.uint64], average: bool = False) -> None:
        """
        Set the color values for the given cells in the elevation map.
        
        Args:
            cells: Cells (N x M array, or 1-dimensional array of size N).
            cmap_index: Column index of the color value in cells (i.e., cells[i, cmap_index] is the color value of i-th cell).
            cell_indices: Indices of the cells in the elevation map (calculated by calc_cell_indices()).
            average: If False, the color values are set directly. Otherwise, the color values of each cell are averaged.
        """
    def set_elevation(self, cells: numpy.ndarray[numpy.float32], elevation_index: int, cell_indices: numpy.ndarray[numpy.uint64], average: bool = False) -> None:
        """
        Set the elevation values for the given cells in the elevation map.
        
        Args:
            cells: Cells (N x M array, or 1-dimensional array of size N).
            elevation_index: Column index of the elevation value in cells (i.e., cells[i, elevation_index] is the elevation value of i-th cell).
            cell_indices: Indices of the cells in the elevation map (calculated by calc_cell_indices()).
            average: If False, the elevation values are set directly. Otherwise, the elevation values of each cell are averaged.
        """
    @property
    def count_map(self) -> numpy.ndarray[numpy.float32[m, n]]:
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
class GLSLShader:
    """
    Wrapper for OpenGL shader programs.
    
    Building a shader program:
        shader = glk.GLSLShader()
        shader.attach_source("vertex_shader.glsl", gl.VERTEX_SHADER)
        shader.attach_source("fragment_shader.glsl", gl.FRAGMENT_SHADER)
        shader.link_program()
    
    Shader source code can also be given as a string with attach_source_code().
    
    Getting attribute and uniform locations:
        shader.use()
        position_loc = shader.attrib("position")
        model_matrix_loc = shader.uniform("model_matrix")
        shader.unuse()
    
    Copying (cached) uniform variables from another shader:
        src_shader.copy_cached_uniforms(dst_shader)
    """
    def __init__(self) -> None:
        ...
    @typing.overload
    def attach_source(self, filename: str, shader_type: int, include_filenames: list[str] = [], defines: str = '', replaces: list[tuple[str, str]] = []) -> bool:
        """
        Attach shader source code from a file, with optional include files, defines, and replacements.
        
        Args:
            filename: Filename of the shader source file to attach.
            shader_type: Type of shader (e.g., gl.VERTEX_SHADER, gl.FRAGMENT_SHADER).
            include_filenames: Filenames of the include files to be inserted into the shader source code.
            defines: Preprocessor definitions to be added to the shader source code.
            replaces: Pairs of strings representing text replacements to be made in the shader source code.
        
        Returns:
            True if the shader source file was successfully attached, False otherwise.
        """
    @typing.overload
    def attach_source(self, filenames: list[str], shader_type: int, include_filenames: list[str] = [], defines: str = '', replaces: list[tuple[str, str]] = []) -> bool:
        """
        Attach shader source code from multiple files, with optional include files, defines, and replacements.
        
        Args:
            filenames: Filenames of the shader source files to attach.
            shader_type: Type of shader (e.g., gl.VERTEX_SHADER, gl.FRAGMENT_SHADER).
            include_filenames: Filenames of the include files to be inserted into the shader source code.
            defines: Preprocessor definitions to be added to the shader source code.
            replaces: Pairs of strings representing text replacements to be made in the shader source code.
        
        Returns:
            True if all shader source files were successfully attached, False otherwise.
        """
    def attach_source_code(self, source_code: str, shader_type: int) -> bool:
        """
        Attach shader source code from a string.
        
        Args:
            source_code: Source code text.
            shader_type: Type of shader (e.g., gl.VERTEX_SHADER, gl.FRAGMENT_SHADER).
        
        Returns:
            True if the shader source code was successfully attached, False otherwise.
        """
    def attrib(self, name: str) -> int:
        ...
    def copy_cached_uniforms(self, other: GLSLShader) -> None:
        """
        Copy the cached uniform variables of this shader to other. other must be in use (other.use()) when calling this.
        """
    def get_uniform4f(self, name: str) -> numpy.ndarray[numpy.float32[4, 1]]:
        ...
    def get_uniform_matrix4f(self, name: str) -> numpy.ndarray[numpy.float32[4, 4]]:
        ...
    def get_uniformf(self, name: str) -> float:
        ...
    def get_uniformi(self, name: str) -> int:
        ...
    def id(self) -> int:
        """
        OpenGL shader program ID.
        """
    @typing.overload
    def init(self, shader_path: str) -> bool:
        ...
    @typing.overload
    def init(self, vertex_shader_path: str, fragment_shader_path: str) -> bool:
        ...
    @typing.overload
    def init(self, vertex_shader_paths: list[str], fragment_shader_paths: list[str]) -> bool:
        ...
    def link_program(self) -> bool:
        """
        Link attached shaders into a shader program. Returns True if linking is successful.
        """
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
        """
        Unuse the shader.
        """
    def use(self) -> None:
        """
        Use the shader in the OpenGL context.
        """
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
    def __init__(self, resolution: float, values: numpy.ndarray[numpy.float32], alpha: float = 1.0, mode: GridMap.ColorMode = ..., scale: float = 1.0) -> None:
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
    def __init__(self, resolution: float, values: numpy.ndarray[numpy.uint8], alpha: int = 255, mode: GridMap.ColorMode = ...) -> None:
        """
        Create a grid map from uint8 values.
        
        Args:
            resolution: Resolution of each grid cell in world units.
            values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
            alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
        """
    @typing.overload
    def update_color(self, values: numpy.ndarray[numpy.float32], alpha: float = 1.0, mode: GridMap.ColorMode = ..., scale: float = 1.0) -> None:
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
    def update_color(self, values: numpy.ndarray[numpy.uint8], alpha: int = 255, mode: GridMap.ColorMode = ...) -> None:
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
    def __init__(self, cloud_buffer: PointCloudBuffer, indices: typing.Any) -> None:
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
    """
    Drawable to draw a set of lines as thick polygons (width in world units).
    """
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
    """
    Options for rendering meshes.
    """
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
    """
    PLY data.
    
    When saving, primary properties (vertices, normals...) are first written, and then the generic properties are written to the file.
    If a property exists in both generic and primary properties, the primary property is prioritized
    (e.g., vertices[:, 0] overwrites properties[name == "x"]).
    """
    colors: numpy.ndarray[numpy.float32[m, n]]
    comments: list[str]
    indices: numpy.ndarray[numpy.int32[m, 1]]
    intensities: numpy.ndarray[numpy.float32[m, 1]]
    normals: numpy.ndarray[numpy.float32[m, n]]
    vertices: numpy.ndarray[numpy.float32[m, n]]
    def __init__(self) -> None:
        ...
    def get_prop(self, name: str) -> PLYGenericPropertyBuffer:
        """
        Get a generic property by name. Returns None if not found.
        """
    @property
    def properties(self) -> list[PLYGenericPropertyBuffer]:
        ...
class PLYGenericPropertyBuffer:
    def as_double(self) -> list[float]:
        ...
    def as_float(self) -> list[float]:
        ...
    def as_int16(self) -> list[int]:
        ...
    def as_int32(self) -> list[int]:
        ...
    def as_int8(self) -> list[str]:
        ...
    def as_uint16(self) -> list[int]:
        ...
    def as_uint32(self) -> list[int]:
        ...
    def as_uint8(self) -> list[int]:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def size(self) -> int:
        ...
    @property
    def type(self) -> PLYPropertyType:
        ...
class PLYPropertyType:
    """
    Members:
    
      CHAR
    
      UCHAR
    
      SHORT
    
      USHORT
    
      INT
    
      UINT
    
      FLOAT
    
      DOUBLE
    """
    CHAR: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.CHAR: 0>
    DOUBLE: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.DOUBLE: 7>
    FLOAT: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.FLOAT: 6>
    INT: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.INT: 4>
    SHORT: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.SHORT: 2>
    UCHAR: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.UCHAR: 1>
    UINT: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.UINT: 5>
    USHORT: typing.ClassVar[PLYPropertyType]  # value = <PLYPropertyType.USHORT: 3>
    __members__: typing.ClassVar[dict[str, PLYPropertyType]]  # value = {'CHAR': <PLYPropertyType.CHAR: 0>, 'UCHAR': <PLYPropertyType.UCHAR: 1>, 'SHORT': <PLYPropertyType.SHORT: 2>, 'USHORT': <PLYPropertyType.USHORT: 3>, 'INT': <PLYPropertyType.INT: 4>, 'UINT': <PLYPropertyType.UINT: 5>, 'FLOAT': <PLYPropertyType.FLOAT: 6>, 'DOUBLE': <PLYPropertyType.DOUBLE: 7>}
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
class PointCloudBuffer(Drawable):
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32]) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    def add_buffer(self, attribute_name: str, data: numpy.ndarray[numpy.float32[m, n]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: numpy.ndarray[numpy.float32]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: list[numpy.ndarray[numpy.float32[4, 1]]]) -> None:
        ...
    def add_colormap(self, cmap: list[float], scale: float = 1.0) -> None:
        ...
    def add_intensity(self, colormap: COLORMAP, intensities: list[float], scale: float = 1.0) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: numpy.ndarray[numpy.float32]) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: list[numpy.ndarray[numpy.float32[3, 1]]]) -> None:
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
        """
        Number of lights.
        """
    def set_albedo(self, albedo: float) -> None:
        ...
    def set_diffuse_model(self, model: ScreenSpaceLighting.DIFFUSE_MODEL) -> None:
        ...
    def set_iridescence_model(self, model: ScreenSpaceLighting.IRIDESCENCE_MODEL) -> None:
        ...
    def set_light(self, i: int, pos: numpy.ndarray[numpy.float32[3, 1]], color: numpy.ndarray[numpy.float32[4, 1]]) -> None:
        """
        Set a point light.
        
        Args:
            i: Light index.
            pos: Light position.
            color: Light color (RGBA).
        """
    def set_occlusion_model(self, model: ScreenSpaceLighting.OCCLUSION_MODEL) -> None:
        ...
    def set_roughness(self, roughness: float) -> None:
        ...
    def set_specular_model(self, model: ScreenSpaceLighting.SPECULAR_MODEL) -> None:
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
    """
    Drawable to draw a set of lines with GL_LINES (fixed width in pixels).
    """
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32], line_strip: bool = False, line_width: float = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: numpy.ndarray[numpy.float32], colors: numpy.ndarray[numpy.float32], line_strip: bool = False, line_width: float = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]], line_strip: bool = False, line_width: float = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: list[numpy.ndarray[numpy.float32[3, 1]]], colors: list[numpy.ndarray[numpy.float32[4, 1]]], line_strip: bool = False, line_width: float = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, x: numpy.ndarray[numpy.float32[m, 1]] = ..., y: numpy.ndarray[numpy.float32[m, 1]] = ..., z: numpy.ndarray[numpy.float32[m, 1]] = ..., c: numpy.ndarray[numpy.float32[m, 1]] = ..., line_strip: bool = True, line_width: float = 1.0) -> None:
        ...
    def set_line_width(self, width: float) -> None:
        ...
class VoxelMap(Drawable):
    """
    Drawable that represents a set of voxels as cubes.
    """
    @typing.overload
    def __init__(self, voxels: numpy.ndarray[numpy.int32], resolution: float, mesh_options: MeshRenderingOptions = ...) -> None:
        """
        Create a voxel map.
        
        Args:
            voxels: Integer coordinates of the voxels (N x 3 array).
            resolution: Size of each voxel (cube) in world units.
            mesh_options: Rendering options for the voxels and edges.
        """
    @typing.overload
    def __init__(self, voxels: list[numpy.ndarray[numpy.int32[3, 1]]], resolution: float, voxel_options: MeshRenderingOptions = ...) -> None:
        """
        Create a voxel map.
        
        Args:
            voxels: List of integer coordinates of the voxels.
            resolution: Size of each voxel (cube) in world units.
            voxel_options: Rendering options for the voxels and edges.
        """
class VoxelMapOptions:
    """
    Deprecated. Use MeshRenderingOptions instead.
    """
    edge_line_width: float
    def __init__(self) -> None:
        ...
    def set_edge_alpha(self, alpha: float) -> None:
        ...
    def set_edge_color(self, color: numpy.ndarray[numpy.float32[4, 1]]) -> None:
        ...
    def set_voxel_alpha(self, alpha: float) -> None:
        ...
    def set_voxel_color(self, color: numpy.ndarray[numpy.float32[4, 1]]) -> None:
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
    """
    Create a point cloud buffer.
    
    Args:
        points: Point positions (N x 3 array).
        colors: Point colors (N x 4 array). Ignored if the number of rows differs from that of points.
    """
def create_texture(image_uint8: numpy.ndarray[numpy.uint8]) -> Texture:
    """
    Create an RGBA8 texture from a uint8 image.
    
    Args:
        image_uint8: Image of shape (height, width) or (height, width, channels).
                     3- and 4-channel images are interpreted as BGR and BGRA (OpenCV order).
    
    Returns:
        Created texture, or None if the image shape is invalid.
    """
def create_texture_f(image_float: numpy.ndarray[numpy.float32]) -> Texture:
    """
    Create an RGBA8 texture from a float image.
    
    Args:
        image_float: Image of shape (height, width) or (height, width, channels) with values in [0, 1].
                     3- and 4-channel images are interpreted as BGR and BGRA (OpenCV order).
    
    Returns:
        Created texture, or None if the image shape is invalid.
    """
def get_data_path() -> str:
    """
    Get the path to the data directory (shaders, fonts, etc.).
    """
def load_ply(filename: str) -> PLYData:
    """
    Load a PLY file.
    
    Args:
        filename: File path.
    
    Returns:
        PLY data if successful, None otherwise.
    """
@typing.overload
def save_ply(filename: str, ply: PLYData, binary: bool = True) -> bool:
    """
    Save a PLY file.
    
    Args:
        filename: File path.
        ply: PLY data.
        binary: If True, save in binary format. Otherwise, save in ASCII format.
    
    Returns:
        True if successful, False otherwise.
    """
@typing.overload
def save_ply(filename: str, points: numpy.ndarray[numpy.float32[m, n]]) -> bool:
    """
    Save points to a binary PLY file.
    
    Args:
        filename: File path.
        points: Point positions (N x 3 or N x 4 array).
    
    Returns:
        True if successful, False otherwise.
    """
def set_data_path(data_path: str) -> None:
    """
    Set the path to the data directory (shaders, fonts, etc.).
    """
AUTUMN: COLORMAP  # value = <COLORMAP.AUTUMN: 6>
BLUE_RED: COLORMAP  # value = <COLORMAP.BLUE_RED: 9>
CHAR: PLYPropertyType  # value = <PLYPropertyType.CHAR: 0>
CIVIDIS: COLORMAP  # value = <COLORMAP.CIVIDIS: 2>
COOL_WARM: COLORMAP  # value = <COLORMAP.COOL_WARM: 17>
CURL: COLORMAP  # value = <COLORMAP.CURL: 16>
DOUBLE: PLYPropertyType  # value = <PLYPropertyType.DOUBLE: 7>
FLOAT: PLYPropertyType  # value = <PLYPropertyType.FLOAT: 6>
GREAN_YELLOW: COLORMAP  # value = <COLORMAP.GREAN_YELLOW: 8>
HELIX: COLORMAP  # value = <COLORMAP.HELIX: 13>
INT: PLYPropertyType  # value = <PLYPropertyType.INT: 4>
JET: COLORMAP  # value = <COLORMAP.JET: 1>
OCEAN: COLORMAP  # value = <COLORMAP.OCEAN: 3>
PASTEL: COLORMAP  # value = <COLORMAP.PASTEL: 12>
PHASE: COLORMAP  # value = <COLORMAP.PHASE: 14>
PUBUGN: COLORMAP  # value = <COLORMAP.PUBUGN: 10>
SHORT: PLYPropertyType  # value = <PLYPropertyType.SHORT: 2>
SPRING: COLORMAP  # value = <COLORMAP.SPRING: 4>
SUMMER: COLORMAP  # value = <COLORMAP.SUMMER: 5>
TURBID: COLORMAP  # value = <COLORMAP.TURBID: 11>
TURBO: COLORMAP  # value = <COLORMAP.TURBO: 0>
UCHAR: PLYPropertyType  # value = <PLYPropertyType.UCHAR: 1>
UINT: PLYPropertyType  # value = <PLYPropertyType.UINT: 5>
USHORT: PLYPropertyType  # value = <PLYPropertyType.USHORT: 3>
VEGETATION: COLORMAP  # value = <COLORMAP.VEGETATION: 15>
WINTER: COLORMAP  # value = <COLORMAP.WINTER: 7>
