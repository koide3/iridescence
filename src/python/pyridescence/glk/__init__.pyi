"""
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
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
    AUTUMN: typing.ClassVar[COLORMAP]
    BLUE_RED: typing.ClassVar[COLORMAP]
    CIVIDIS: typing.ClassVar[COLORMAP]
    COOL_WARM: typing.ClassVar[COLORMAP]
    CURL: typing.ClassVar[COLORMAP]
    GREAN_YELLOW: typing.ClassVar[COLORMAP]
    HELIX: typing.ClassVar[COLORMAP]
    JET: typing.ClassVar[COLORMAP]
    OCEAN: typing.ClassVar[COLORMAP]
    PASTEL: typing.ClassVar[COLORMAP]
    PHASE: typing.ClassVar[COLORMAP]
    PUBUGN: typing.ClassVar[COLORMAP]
    SPRING: typing.ClassVar[COLORMAP]
    SUMMER: typing.ClassVar[COLORMAP]
    TURBID: typing.ClassVar[COLORMAP]
    TURBO: typing.ClassVar[COLORMAP]
    VEGETATION: typing.ClassVar[COLORMAP]
    WINTER: typing.ClassVar[COLORMAP]
    __members__: typing.ClassVar[dict[str, COLORMAP]]
    @typing.overload
    def __eq__(self, other: COLORMAP) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: COLORMAP) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
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
    def attach_source(self, filename: str, shader_type: typing.SupportsInt | typing.SupportsIndex, include_filenames: collections.abc.Sequence[str] = [], defines: str = '', replaces: collections.abc.Sequence[tuple[str, str]] = []) -> bool:
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
    def attach_source(self, filenames: collections.abc.Sequence[str], shader_type: typing.SupportsInt | typing.SupportsIndex, include_filenames: collections.abc.Sequence[str] = [], defines: str = '', replaces: collections.abc.Sequence[tuple[str, str]] = []) -> bool:
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
    def attach_source_code(self, source_code: str, shader_type: typing.SupportsInt | typing.SupportsIndex) -> bool:
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
    def get_uniform4f(self, name: str) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 1]"]:
        ...
    def get_uniform_matrix4f(self, name: str) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]:
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
    def init(self, vertex_shader_paths: collections.abc.Sequence[str], fragment_shader_paths: collections.abc.Sequence[str]) -> bool:
        ...
    def link_program(self) -> bool:
        """
        Link attached shaders into a shader program. Returns True if linking is successful.
        """
    @typing.overload
    def set_uniform(self, name: str, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[3, 1]"]) -> None:
        ...
    @typing.overload
    def set_uniform(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[4, 1]"]) -> None:
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
class Drawable:
    def __init__(self) -> None:
        ...
    def draw(self, shader: GLSLShader) -> None:
        ...
class ThinLines(Drawable):
    """
    Drawable to draw a set of lines with GL_LINES (fixed width in pixels).
    """
    @typing.overload
    def __init__(self, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], colors: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    @typing.overload
    def __init__(self, x: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., y: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., z: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., c: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., line_strip: bool = True, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    def set_line_width(self, width: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class Lines(Drawable):
    """
    Drawable to draw a set of lines as thick polygons (width in world units).
    """
    @typing.overload
    def __init__(self, thickness: typing.SupportsFloat | typing.SupportsIndex, vertices: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], colors: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]]) -> None:
        ...
    @typing.overload
    def __init__(self, thickness: typing.SupportsFloat | typing.SupportsIndex, vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        ...
    @typing.overload
    def __init__(self, x: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., y: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., z: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., c: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"] = ..., width: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612, line_strip: bool = True) -> None:
        ...
class PointCloudBuffer(Drawable):
    @typing.overload
    def __init__(self, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        ...
    @typing.overload
    def __init__(self, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]]) -> None:
        ...
    def add_buffer(self, attribute_name: str, data: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]]) -> None:
        ...
    @typing.overload
    def add_color(self, colors: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]]) -> None:
        ...
    def add_colormap(self, cmap: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], scale: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    def add_intensity(self, colormap: COLORMAP, intensities: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], scale: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> None:
        ...
    @typing.overload
    def add_normals(self, normals: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]]) -> None:
        ...
    def set_colormap_buffer(self, colormap_buffer: str) -> None:
        ...
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
class PointNormalsBuffer(Drawable):
    """
    Drawable that visualizes point normals as line segments.
    Only the normal lines (from p to p + n * normal_length) are drawn. Points themselves are not rendered.
    """
    @typing.overload
    def __init__(self, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], normals: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], normal_length: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        """
        Create a point normals buffer.
        
        Args:
            points: Point positions (N x 3 or N x 4 array). The 4th column is ignored.
            normals: Normal vectors (N x 3 or N x 4 array). The 4th column is ignored.
            normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
        """
    @typing.overload
    def __init__(self, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], normals: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], normal_length: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
        """
        Create a point normals buffer.
        
        Args:
            points: List of point positions.
            normals: List of normal vectors. Must have the same size as points.
            normal_length: Scale factor applied to each normal. It equals the drawn line length for unit normals.
        """
class VoxelMapOptions:
    """
    Deprecated. Use MeshRenderingOptions instead.
    """
    def __init__(self) -> None:
        ...
    def set_edge_alpha(self, alpha: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_edge_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        ...
    def set_voxel_alpha(self, alpha: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_voxel_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        ...
    @property
    def edge_line_width(self) -> float:
        ...
    @edge_line_width.setter
    def edge_line_width(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class MeshRenderingOptions:
    """
    Options for rendering meshes.
    """
    draw_edges: bool
    draw_faces: bool
    def __init__(self) -> None:
        ...
    def set_edge_alpha(self, alpha: typing.SupportsFloat | typing.SupportsIndex) -> MeshRenderingOptions:
        ...
    def set_edge_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> MeshRenderingOptions:
        ...
    def set_face_alpha(self, alpha: typing.SupportsFloat | typing.SupportsIndex) -> MeshRenderingOptions:
        ...
    def set_face_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> MeshRenderingOptions:
        ...
    @property
    def edge_line_width(self) -> float:
        ...
    @edge_line_width.setter
    def edge_line_width(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
class VoxelMap(Drawable):
    """
    Drawable that represents a set of voxels as cubes.
    """
    @typing.overload
    def __init__(self, voxels: typing.Annotated[numpy.typing.ArrayLike, numpy.int32], resolution: typing.SupportsFloat | typing.SupportsIndex, mesh_options: MeshRenderingOptions = ...) -> None:
        """
        Create a voxel map.
        
        Args:
            voxels: Integer coordinates of the voxels (N x 3 array).
            resolution: Size of each voxel (cube) in world units.
            mesh_options: Rendering options for the voxels and edges.
        """
    @typing.overload
    def __init__(self, voxels: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[3, 1]"]], resolution: typing.SupportsFloat | typing.SupportsIndex, voxel_options: MeshRenderingOptions = ...) -> None:
        """
        Create a voxel map.
        
        Args:
            voxels: List of integer coordinates of the voxels.
            resolution: Size of each voxel (cube) in world units.
            voxel_options: Rendering options for the voxels and edges.
        """
class ElevationMapBuilder:
    """
    Builder for an elevation map.
    """
    def __init__(self, resolution: typing.SupportsFloat | typing.SupportsIndex, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], origin: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]) -> None:
        """
        Create an elevation map builder.
        
        Args:
            resolution: Resolution of the map (size of each cell).
            size: Size of the map in number of cells (width, height).
            origin: Top-left corner of the map in world coordinates.
        """
    def calc_cell_indices(self, cells: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> numpy.typing.NDArray[numpy.uint64]:
        """
        Calculate the cell indices for a given set of cells.
        
        Args:
            cells: Cells for which to calculate the indices (N x M array, M >= 2). cells[i, 0] and cells[i, 1] are the x and y coordinates of the cell in world coordinates.
        
        Returns:
            Indices of the cells in the elevation map.
        """
    def calc_cmap_gradient(self, window_size: typing.SupportsInt | typing.SupportsIndex = 3) -> None:
        """
        Replace the color map with its gradient magnitude.
        
        Args:
            window_size: Size of the filter window (3, 5, or 7).
        """
    def filter_elevation_map(self, process_valid_cells: bool = True, window_size: typing.SupportsInt | typing.SupportsIndex = 3) -> None:
        """
        Apply local averaging filter to the elevation (and color) map.
        
        Args:
            process_valid_cells: If True, all cells are processed (smoothed). If False, only invalid cells (NaN) are processed.
            window_size: Size of the filter window (3, 5, or 7).
        """
    def set_cmap(self, cells: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], cmap_index: typing.SupportsInt | typing.SupportsIndex, cell_indices: typing.Annotated[numpy.typing.ArrayLike, numpy.uint64], average: bool = False) -> None:
        """
        Set the color values for the given cells in the elevation map.
        
        Args:
            cells: Cells (N x M array, or 1-dimensional array of size N).
            cmap_index: Column index of the color value in cells (i.e., cells[i, cmap_index] is the color value of i-th cell).
            cell_indices: Indices of the cells in the elevation map (calculated by calc_cell_indices()).
            average: If False, the color values are set directly. Otherwise, the color values of each cell are averaged.
        """
    def set_elevation(self, cells: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], elevation_index: typing.SupportsInt | typing.SupportsIndex, cell_indices: typing.Annotated[numpy.typing.ArrayLike, numpy.uint64], average: bool = False) -> None:
        """
        Set the elevation values for the given cells in the elevation map.
        
        Args:
            cells: Cells (N x M array, or 1-dimensional array of size N).
            elevation_index: Column index of the elevation value in cells (i.e., cells[i, elevation_index] is the elevation value of i-th cell).
            cell_indices: Indices of the cells in the elevation map (calculated by calc_cell_indices()).
            average: If False, the elevation values are set directly. Otherwise, the elevation values of each cell are averaged.
        """
    @property
    def color_map(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @color_map.setter
    def color_map(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
    @property
    def count_map(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @property
    def elevation_map(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @elevation_map.setter
    def elevation_map(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
    @property
    def origin(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[2, 1]"]:
        ...
    @property
    def resolution(self) -> float:
        ...
    @property
    def size(self) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]:
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
        PROB: typing.ClassVar[GridMap.ColorMode]
        PROB_TURBO: typing.ClassVar[GridMap.ColorMode]
        RAW: typing.ClassVar[GridMap.ColorMode]
        RGBA: typing.ClassVar[GridMap.ColorMode]
        TURBO: typing.ClassVar[GridMap.ColorMode]
        __members__: typing.ClassVar[dict[str, GridMap.ColorMode]]
        @typing.overload
        def __eq__(self, other: GridMap.ColorMode) -> bool:
            ...
        @typing.overload
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __int__(self) -> int:
            ...
        @typing.overload
        def __ne__(self, other: GridMap.ColorMode) -> bool:
            ...
        @typing.overload
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    PROB: typing.ClassVar[GridMap.ColorMode]
    PROB_TURBO: typing.ClassVar[GridMap.ColorMode]
    RAW: typing.ClassVar[GridMap.ColorMode]
    RGBA: typing.ClassVar[GridMap.ColorMode]
    TURBO: typing.ClassVar[GridMap.ColorMode]
    @typing.overload
    def __init__(self, resolution: typing.SupportsFloat | typing.SupportsIndex, values: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], alpha: typing.SupportsFloat | typing.SupportsIndex = 1.0, mode: GridMap.ColorMode = ..., scale: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
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
    def __init__(self, resolution: typing.SupportsFloat | typing.SupportsIndex, values: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], alpha: typing.SupportsInt | typing.SupportsIndex = 255, mode: GridMap.ColorMode = ...) -> None:
        """
        Create a grid map from uint8 values.
        
        Args:
            resolution: Resolution of each grid cell in world units.
            values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
            alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
        """
    @typing.overload
    def update_color(self, values: typing.Annotated[numpy.typing.ArrayLike, numpy.float32], alpha: typing.SupportsFloat | typing.SupportsIndex = 1.0, mode: GridMap.ColorMode = ..., scale: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> None:
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
    def update_color(self, values: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8], alpha: typing.SupportsInt | typing.SupportsIndex = 255, mode: GridMap.ColorMode = ...) -> None:
        """
        Update the colors with uint8 values.
        
        The map extent given at construction does not change. If the shape of values differs from the original one, the colors are stretched to fit the extent.
        
        Args:
            values: Color values (uint8) of shape (height, width), or (height, width, 4) for RGBA mode.
            alpha: Alpha value for the entire map [0, 255]. Ignored if mode is RGBA.
            mode: Color mode for interpreting the values.
        """
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
    def __init__(self, drawables: collections.abc.Sequence[typing.Any], skip_model_matrix_setting: bool = True) -> None:
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
class Texture:
    """
    2D OpenGL texture.
    All methods must be called in the thread that owns the OpenGL context.
    Pixel data are assumed to be tightly packed (no row padding).
    """
    @typing.overload
    def __init__(self, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], internal_format: typing.SupportsInt | typing.SupportsIndex, format: typing.SupportsInt | typing.SupportsIndex, type: typing.SupportsInt | typing.SupportsIndex, bytes: collections.abc.Buffer) -> None:
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
    def __init__(self, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], internal_format: typing.SupportsInt | typing.SupportsIndex, format: typing.SupportsInt | typing.SupportsIndex, type: typing.SupportsInt | typing.SupportsIndex, bytes: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex]) -> None:
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
    def bind(self, unit: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Bind the texture to the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.
        """
    def id(self) -> int:
        """
        OpenGL texture ID. Note that it changes when set_size() is called.
        """
    def read_pixels(self, format: typing.SupportsInt | typing.SupportsIndex = 6408, type: typing.SupportsInt | typing.SupportsIndex = 5121, num_components: typing.SupportsInt | typing.SupportsIndex = -1) -> numpy.ndarray:
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
    def set_clamp_mode(self, mode: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Set the wrap mode for both S and T (e.g., gl.CLAMP_TO_EDGE, gl.REPEAT).
        """
    def set_filter_mode(self, mode: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Set the min/mag filter mode (gl.LINEAR or gl.NEAREST).
        """
    def set_size(self, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]) -> None:
        """
        Change the size of the texture.
        The texture is re-created: the texture data are discarded, the texture ID changes, and the filter and clamp modes are reset to the defaults.
        Do not call this on the viewer's frame buffer textures (e.g., color_buffer()).
        """
    def size(self) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]:
        """
        Texture size (width, height).
        """
    @typing.overload
    def unbind(self) -> None:
        """
        Unbind GL_TEXTURE_2D of the active texture unit.
        """
    @typing.overload
    def unbind(self, unit: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Unbind GL_TEXTURE_2D of the specified texture unit (e.g., gl.TEXTURE0 + 1). The active texture unit is set to gl.TEXTURE0 afterwards.
        """
class ScreenEffect:
    pass
class NaiveScreenSpaceAmbientOcclusion(ScreenEffect):
    def __init__(self) -> None:
        ...
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
        DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
        LAMBERT: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
        NORMALIZED_DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
        OREN_NAYAR: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
        ZERO: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.DIFFUSE_MODEL]]
        @typing.overload
        def __eq__(self, other: ScreenSpaceLighting.DIFFUSE_MODEL) -> bool:
            ...
        @typing.overload
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __int__(self) -> int:
            ...
        @typing.overload
        def __ne__(self, other: ScreenSpaceLighting.DIFFUSE_MODEL) -> bool:
            ...
        @typing.overload
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
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
        BLINN_PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
        COOK_TORRANCE: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
        PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
        ZERO: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.SPECULAR_MODEL]]
        @typing.overload
        def __eq__(self, other: ScreenSpaceLighting.SPECULAR_MODEL) -> bool:
            ...
        @typing.overload
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __int__(self) -> int:
            ...
        @typing.overload
        def __ne__(self, other: ScreenSpaceLighting.SPECULAR_MODEL) -> bool:
            ...
        @typing.overload
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
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
        AMBIENT_OCCLUSION: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]
        ZERO: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.OCCLUSION_MODEL]]
        @typing.overload
        def __eq__(self, other: ScreenSpaceLighting.OCCLUSION_MODEL) -> bool:
            ...
        @typing.overload
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __int__(self) -> int:
            ...
        @typing.overload
        def __ne__(self, other: ScreenSpaceLighting.OCCLUSION_MODEL) -> bool:
            ...
        @typing.overload
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
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
        IRIDESCENCE1: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
        IRIDESCENCE2: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
        IRIDESCENCE3: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
        ZERO: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
        __members__: typing.ClassVar[dict[str, ScreenSpaceLighting.IRIDESCENCE_MODEL]]
        @typing.overload
        def __eq__(self, other: ScreenSpaceLighting.IRIDESCENCE_MODEL) -> bool:
            ...
        @typing.overload
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __int__(self) -> int:
            ...
        @typing.overload
        def __ne__(self, other: ScreenSpaceLighting.IRIDESCENCE_MODEL) -> bool:
            ...
        @typing.overload
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    AMBIENT_OCCLUSION: typing.ClassVar[ScreenSpaceLighting.OCCLUSION_MODEL]
    BLINN_PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
    COOK_TORRANCE: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
    DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
    IRIDESCENCE1: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
    IRIDESCENCE2: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
    IRIDESCENCE3: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
    LAMBERT: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
    NORMALIZED_DISNEY: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
    OREN_NAYAR: typing.ClassVar[ScreenSpaceLighting.DIFFUSE_MODEL]
    PHONG: typing.ClassVar[ScreenSpaceLighting.SPECULAR_MODEL]
    ZERO: typing.ClassVar[ScreenSpaceLighting.IRIDESCENCE_MODEL]
    def __init__(self) -> None:
        ...
    def num_lights(self) -> int:
        """
        Number of lights.
        """
    def set_albedo(self, albedo: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_diffuse_model(self, model: ScreenSpaceLighting.DIFFUSE_MODEL) -> None:
        ...
    def set_iridescence_model(self, model: ScreenSpaceLighting.IRIDESCENCE_MODEL) -> None:
        ...
    def set_light(self, i: typing.SupportsInt | typing.SupportsIndex, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        """
        Set a point light.
        
        Args:
            i: Light index.
            pos: Light position.
            color: Light color (RGBA).
        """
    def set_occlusion_model(self, model: ScreenSpaceLighting.OCCLUSION_MODEL) -> None:
        ...
    def set_roughness(self, roughness: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    def set_specular_model(self, model: ScreenSpaceLighting.SPECULAR_MODEL) -> None:
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
    CHAR: typing.ClassVar[PLYPropertyType]
    DOUBLE: typing.ClassVar[PLYPropertyType]
    FLOAT: typing.ClassVar[PLYPropertyType]
    INT: typing.ClassVar[PLYPropertyType]
    SHORT: typing.ClassVar[PLYPropertyType]
    UCHAR: typing.ClassVar[PLYPropertyType]
    UINT: typing.ClassVar[PLYPropertyType]
    USHORT: typing.ClassVar[PLYPropertyType]
    __members__: typing.ClassVar[dict[str, PLYPropertyType]]
    @typing.overload
    def __eq__(self, other: PLYPropertyType) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __int__(self) -> int:
        ...
    @typing.overload
    def __ne__(self, other: PLYPropertyType) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
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
class PLYData:
    """
    PLY data.
    
    When saving, primary properties (vertices, normals...) are first written, and then the generic properties are written to the file.
    If a property exists in both generic and primary properties, the primary property is prioritized
    (e.g., vertices[:, 0] overwrites properties[name == "x"]).
    """
    def __init__(self) -> None:
        ...
    def get_prop(self, name: str) -> PLYGenericPropertyBuffer:
        """
        Get a generic property by name. Returns None if not found.
        """
    @property
    def colors(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @colors.setter
    def colors(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
    @property
    def comments(self) -> list[str]:
        ...
    @comments.setter
    def comments(self, arg1: collections.abc.Sequence[str]) -> None:
        ...
    @property
    def indices(self) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[m, 1]"]:
        ...
    @indices.setter
    def indices(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[m, 1]"]) -> None:
        ...
    @property
    def intensities(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, 1]"]:
        ...
    @intensities.setter
    def intensities(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"]) -> None:
        ...
    @property
    def normals(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @normals.setter
    def normals(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
    @property
    def properties(self) -> list[PLYGenericPropertyBuffer]:
        ...
    @property
    def vertices(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
        ...
    @vertices.setter
    def vertices(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> None:
        ...
def colormap(colormap_type: COLORMAP, x: typing.SupportsInt | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[4, 1]"]:
    ...
def colormap_categorical(colormap_type: COLORMAP, x: typing.SupportsInt | typing.SupportsIndex, num_categories: typing.SupportsInt | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[4, 1]"]:
    ...
def colormap_categoricalf(colormap_type: COLORMAP, x: typing.SupportsInt | typing.SupportsIndex, num_categories: typing.SupportsInt | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 1]"]:
    ...
def colormapf(colormap_type: COLORMAP, x: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 1]"]:
    ...
def create_pointcloud_buffer(points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 3]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 4]"] = ...) -> PointCloudBuffer:
    """
    Create a point cloud buffer.
    
    Args:
        points: Point positions (N x 3 array).
        colors: Point colors (N x 4 array). Ignored if the number of rows differs from that of points.
    """
def create_texture(image_uint8: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8]) -> Texture:
    """
    Create an RGBA8 texture from a uint8 image.
    
    Args:
        image_uint8: Image of shape (height, width) or (height, width, channels).
                     3- and 4-channel images are interpreted as BGR and BGRA (OpenCV order).
    
    Returns:
        Created texture, or None if the image shape is invalid.
    """
def create_texture_f(image_float: typing.Annotated[numpy.typing.ArrayLike, numpy.float32]) -> Texture:
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
def save_ply(filename: str, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"]) -> bool:
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
AUTUMN: COLORMAP
BLUE_RED: COLORMAP
CHAR: PLYPropertyType
CIVIDIS: COLORMAP
COOL_WARM: COLORMAP
CURL: COLORMAP
DOUBLE: PLYPropertyType
FLOAT: PLYPropertyType
GREAN_YELLOW: COLORMAP
HELIX: COLORMAP
INT: PLYPropertyType
JET: COLORMAP
OCEAN: COLORMAP
PASTEL: COLORMAP
PHASE: COLORMAP
PUBUGN: COLORMAP
SHORT: PLYPropertyType
SPRING: COLORMAP
SUMMER: COLORMAP
TURBID: COLORMAP
TURBO: COLORMAP
UCHAR: PLYPropertyType
UINT: PLYPropertyType
USHORT: PLYPropertyType
VEGETATION: COLORMAP
WINTER: COLORMAP
