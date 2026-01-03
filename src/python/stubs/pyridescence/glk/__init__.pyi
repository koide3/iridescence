"""
"""
from __future__ import annotations
import numpy
import typing
from . import primitives
__all__: list[str] = ['AUTUMN', 'BLUE_RED', 'CIVIDIS', 'COLORMAP', 'COOL_WARM', 'CURL', 'Drawable', 'GREAN_YELLOW', 'HELIX', 'JET', 'Lines', 'NaiveScreenSpaceAmbientOcclusion', 'OCEAN', 'PASTEL', 'PHASE', 'PLYData', 'PUBUGN', 'PointCloudBuffer', 'SPRING', 'SUMMER', 'ScreenEffect', 'ScreenSpaceAmbientOcclusion', 'ScreenSpaceLighting', 'TURBID', 'TURBO', 'Texture', 'ThinLines', 'VEGETATION', 'WINTER', 'colormap', 'colormap_categorical', 'colormap_categoricalf', 'colormapf', 'create_pointcloud_buffer', 'create_texture', 'create_texture_f', 'get_data_path', 'load_ply', 'primitives', 'save_ply', 'set_data_path']
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
    pass
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
    def __init__(self, size: numpy.ndarray[numpy.int32[2, 1]], internal_format: int, format: int, type: int, bytes: list[int]) -> None:
        ...
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
