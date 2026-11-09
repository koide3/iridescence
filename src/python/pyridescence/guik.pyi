"""
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import pyridescence.glk
import typing
__all__: list[str] = ['Application', 'ArcBallCameraControl', 'AsyncLightViewer', 'AsyncLightViewerContext', 'BasicProjectionControl', 'CIRCLE', 'CameraControl', 'FPSCameraControl', 'FlatBlack', 'FlatBlue', 'FlatColor', 'FlatDarkGray', 'FlatGray', 'FlatGreen', 'FlatOrange', 'FlatRed', 'FlatWhite', 'HoveredDrawings', 'LightViewer', 'LightViewerContext', 'METRIC', 'ModelControl', 'OrbitCameraControlXY', 'OrbitCameraControlXZ', 'PointScaleMode', 'PointShapeMode', 'ProjectionControl', 'RECTANGLE', 'Rainbow', 'RecentFiles', 'SCREENSPACE', 'SensorViewCameraControl', 'ShaderSetting', 'TextureColor', 'TopDownCameraControl', 'VertexColor', 'anon', 'async_destroy', 'async_running', 'async_viewer', 'async_wait', 'destroy', 'running', 'viewer']
class PointShapeMode:
    """
    Point shape mode.
    
    RECTANGLE: Square/rectangle shape
    CIRCLE: Circle shape
    
    
    Members:
    
      RECTANGLE
    
      CIRCLE
    """
    CIRCLE: typing.ClassVar[PointShapeMode]
    RECTANGLE: typing.ClassVar[PointShapeMode]
    __members__: typing.ClassVar[dict[str, PointShapeMode]]
    @typing.overload
    def __eq__(self, other: PointShapeMode) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.SupportsInt | typing.SupportsIndex) -> bool:
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
    def __ne__(self, other: PointShapeMode) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.SupportsInt | typing.SupportsIndex) -> bool:
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
class PointScaleMode:
    """
    Point size scale mode.
    
    SCREENSPACE: Screen space size
    METRIC: Metric size (in meters)
    
    
    Members:
    
      SCREENSPACE
    
      METRIC
    """
    METRIC: typing.ClassVar[PointScaleMode]
    SCREENSPACE: typing.ClassVar[PointScaleMode]
    __members__: typing.ClassVar[dict[str, PointScaleMode]]
    @typing.overload
    def __eq__(self, other: PointScaleMode) -> bool:
        ...
    @typing.overload
    def __eq__(self, other: typing.SupportsInt | typing.SupportsIndex) -> bool:
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
    def __ne__(self, other: PointScaleMode) -> bool:
        ...
    @typing.overload
    def __ne__(self, other: typing.SupportsInt | typing.SupportsIndex) -> bool:
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
class ShaderSetting:
    """
    Shader setting that holds rendering settings (color mode, model matrix, point shape, etc.).
    
    Setter methods return the shader setting itself so that they can be chained (e.g., guik.FlatRed().set_point_size(2.0).translate(1.0, 0.0, 0.0)).
    """
    @typing.overload
    def add(self, name: str, value: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.
        
        Args:
            name: Parameter name (uniform variable name in the shader).
            value: Parameter value.
        """
    @typing.overload
    def add(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]) -> ShaderSetting:
        """
        Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.
        
        Args:
            name: Parameter name (uniform variable name in the shader).
            value: Parameter value.
        """
    @typing.overload
    def add(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> ShaderSetting:
        """
        Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.
        
        Args:
            name: Parameter name (uniform variable name in the shader).
            value: Parameter value.
        """
    def addi(self, name: str, value: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[4, 1]"]) -> ShaderSetting:
        """
        Add a new integer vector (ivec4) parameter to the shader setting. If a parameter with the same name exists, it is overwritten.
        
        Args:
            name: Parameter name (uniform variable name in the shader).
            value: Parameter value.
        """
    def color_mode(self) -> int:
        """
        Get color mode.
        """
    def dynamic_object(self) -> ShaderSetting:
        """
        Set dynamic object flag.
        """
    def make_transparent(self) -> ShaderSetting:
        """
        Make the object transparent (alpha blending).
        """
    def material_color(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 1]"]:
        """
        Get material color (for FLAT_COLOR).
        """
    def model_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]:
        """
        Get model matrix.
        """
    def point_scale(self) -> float:
        """
        Get point size scale factor.
        """
    def remove_model_matrix(self) -> ShaderSetting:
        """
        Remove the model matrix. Note that this invalidates subsequent model matrix operations (translate, rotate, and scale).
        """
    @typing.overload
    def rotate(self, angle: typing.SupportsFloat | typing.SupportsIndex, axis: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> ShaderSetting:
        """
        Apply rotation (angle [rad] around axis) to the model matrix.
        """
    @typing.overload
    def rotate(self, rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> ShaderSetting:
        """
        Apply rotation (3x3 rotation matrix) to the model matrix.
        """
    def rotation(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        """
        Get rotation matrix from the model matrix.
        """
    @typing.overload
    def scale(self, scale: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Apply uniform scaling to the model matrix.
        """
    @typing.overload
    def scale(self, scale: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> ShaderSetting:
        """
        Apply non-uniform scaling to the model matrix.
        """
    def set_alpha(self, alpha: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Set alpha value of the material color. If alpha < 0.999, the object is rendered as transparent.
        """
    @typing.overload
    def set_color(self, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> ShaderSetting:
        """
        Set material color (for FLAT_COLOR). If a < 0.999, the object is rendered as transparent.
        """
    @typing.overload
    def set_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> ShaderSetting:
        """
        Set material color (RGBA, for FLAT_COLOR). If alpha < 0.999, the object is rendered as transparent.
        """
    def set_color_mode(self, color_mode: typing.SupportsInt | typing.SupportsIndex) -> ShaderSetting:
        """
        Set color mode (0: RAINBOW, 1: FLAT_COLOR, 2: VERTEX_COLOR, 3: TEXTURE_COLOR, 4: VERTEX_COLORMAP).
        """
    def set_model_matrix(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> ShaderSetting:
        """
        Set model matrix.
        """
    def set_point_scale(self, scale: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Set point size scale factor.
        """
    def set_point_scale_metric(self) -> ShaderSetting:
        """
        Set point size scale to metric.
        """
    def set_point_scale_mode(self, mode: PointScaleMode) -> ShaderSetting:
        """
        Set point size scale mode.
        """
    def set_point_scale_screenspace(self) -> ShaderSetting:
        """
        Set point size scale to screen space.
        """
    def set_point_shape(self, point_size: typing.SupportsFloat | typing.SupportsIndex, metric: bool, circle: bool) -> ShaderSetting:
        """
        Set point shape with common settings.
        
        Args:
            point_size: Point size in meters (if metric) or in pseudo pixels (if screenspace).
            metric: If True, point size is in meters. If False, point size is in pixels.
            circle: If True, point shape is circle. If False, point shape is rectangle.
        """
    def set_point_shape_circle(self) -> ShaderSetting:
        """
        Set point shape to circle.
        """
    def set_point_shape_mode(self, mode: PointShapeMode) -> ShaderSetting:
        """
        Set point shape mode.
        """
    def set_point_shape_rectangle(self) -> ShaderSetting:
        """
        Set point shape to rectangle.
        """
    def set_point_size(self, size: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Set point size.
        """
    def set_point_size_offset(self, offset: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Set point size offset.
        """
    def static_object(self) -> ShaderSetting:
        """
        Set static object flag.
        """
    def transform(self, transform: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> ShaderSetting:
        """
        Apply transformation to the model matrix. The transformation is applied after the existing transformation (T_new = T_old * transform).
        """
    @typing.overload
    def translate(self, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, z: typing.SupportsFloat | typing.SupportsIndex) -> ShaderSetting:
        """
        Apply translation to the model matrix.
        """
    @typing.overload
    def translate(self, translation: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> ShaderSetting:
        """
        Apply translation to the model matrix.
        """
    def translation(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Get translation vector from the model matrix.
        """
class Rainbow(ShaderSetting):
    """
    Rainbow coloring scheme that assigns colors to the height (z-value) of pixels of the object.
    The coloring band can be changed by setting `z_range` param (e.g., add("z_range", [-3.0, 5.0])).
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class VertexColor(ShaderSetting):
    """
    Vertex color scheme that assigns colors based on vertex color attributes.
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class TextureColor(ShaderSetting):
    """
    Texture color scheme that assigns colors based on the bound texture.
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatColor(ShaderSetting):
    """
    Flat coloring scheme that assigns a single color to the object.
    If the alpha value is less than 0.999, the object is rendered as transparent.
    """
    @typing.overload
    def __init__(self, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...
    @typing.overload
    def __init__(self, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatRed(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatGreen(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatBlue(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatOrange(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatWhite(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatGray(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatDarkGray(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class FlatBlack(FlatColor):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        ...
    @typing.overload
    def __init__(self, scale: typing.SupportsFloat | typing.SupportsIndex = 1.0, trans: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"] = ..., rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"] = ...) -> None:
        ...
class ModelControl:
    """
    UI and gizmo to manipulate a model matrix.
    """
    def __init__(self, label: str = 'model_control', model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"] = ...) -> None:
        ...
    def disable_gizmo(self) -> None:
        ...
    def draw_gizmo(self) -> None:
        """
        Draw the gizmo on the main viewer canvas. Call this in a UI callback.
        """
    def draw_gizmo_ui(self) -> None:
        """
        Draw the gizmo setting UI. Call this in a UI callback.
        """
    def draw_ui(self) -> None:
        """
        Draw the model control UI window. Call this in a UI callback.
        """
    def enable_gizmo(self) -> None:
        ...
    def model_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]:
        """
        Get the model matrix.
        """
    def set_gizmo_clip_scale(self, space: typing.SupportsFloat | typing.SupportsIndex = 0.10000000149011612) -> None:
        """
        Change the gizmo size (default = 0.1).
        """
    def set_gizmo_enabled(self, enabled: bool) -> None:
        ...
    def set_gizmo_mode(self, mode: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Set the gizmo mode (0: LOCAL, 1: WORLD).
        """
    def set_gizmo_operation(self, operation: str) -> None:
        """
        Set the gizmo operation.
        
        Args:
            operation: "TRANSLATE", "ROTATE", "SCALE", "SCALEU", or "UNIVERSAL".
        """
    def set_model_matrix(self, model_matrix: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        """
        Set the model matrix.
        """
class CameraControl:
    """
    Base class for camera control.
    
    Many of default camera control methods have an interface that combines a programmatic API (e.g., lookat()) and
    user input (e.g., mouse dragging). For example, lookat() can be used to programmatically set the camera to look at
    a specific point in the world, while mouse dragging can be used to shift the camera's center of focus.
    reset_center() can be used to reset any such shifts.
    """
    @typing.overload
    def lookat(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
        Move the camera to look at the given point in world coordinates.
        """
    @typing.overload
    def lookat(self, T_world_sensor: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]) -> None:
        """
        Move the camera to look at the given transform (4x4 matrix in world coordinates).
        The camera looks at the translation part of the transform, and optionally aligns its orientation with the rotation part.
        """
    def reset_center(self) -> None:
        """
        Reset the shift of the camera.
        """
    def view_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]:
        """
        Get the 4x4 view matrix of the camera.
        """
class OrbitCameraControlXY(CameraControl):
    pass
class OrbitCameraControlXZ(CameraControl):
    pass
class ArcBallCameraControl(CameraControl):
    pass
class TopDownCameraControl(CameraControl):
    pass
class FPSCameraControl(CameraControl):
    """
    FPS game-like camera control.
    """
    def set_depth_range(self, range: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"]) -> None:
        """
        Set the depth range (near, far).
        """
    def set_fovy(self, fovy_deg: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Set the vertical field of view [deg].
        """
    def set_mouse_senstivity(self, mouse_sensitivity_pitch: typing.SupportsFloat | typing.SupportsIndex = 0.01, mouse_sensitivity_yaw: typing.SupportsFloat | typing.SupportsIndex = 0.01) -> None:
        """
        Set the mouse sensitivity.
        """
    def set_pose(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], yaw_deg: typing.SupportsFloat | typing.SupportsIndex, pitch_deg: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Set the camera pose.
        """
    def set_translation_speed(self, speed: typing.SupportsFloat | typing.SupportsIndex = 0.05) -> None:
        """
        Set the translation speed.
        """
class SensorViewCameraControl(CameraControl):
    """
    Camera control that orbits around a tracked sensor pose.
    The camera always looks at the sensor position, with user-controlled azimuth/elevation and distance scaling.
    """
    @staticmethod
    def set_sensor_camera_transform(*args, **kwargs) -> None:
        """
        Set the transform from sensor frame to camera frame (T_sensor_camera, 4x4 matrix).
        """
    @staticmethod
    def set_sensor_pose(*args, **kwargs) -> None:
        """
        Set the raw sensor pose (T_world_sensor, 4x4 matrix).
        """
    def set_smoothing_factor(self, trans: typing.SupportsFloat | typing.SupportsIndex, rot: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Set translation and rotation smoothing factors in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
        """
    def set_z_axis_alignment(self, enabled: bool) -> None:
        """
        Enable or disable z-axis alignment. When enabled (default), the camera up direction is locked to the world z-axis.
        """
class ProjectionControl:
    pass
class BasicProjectionControl(ProjectionControl):
    pass
class HoveredDrawings:
    def __init__(self, context: ... = None) -> None:
        ...
    def add_circle(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, radius: typing.SupportsFloat | typing.SupportsIndex = 10.0, num_segments: typing.SupportsInt | typing.SupportsIndex = 32, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_circle_on(self, drawable_name: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, radius: typing.SupportsFloat | typing.SupportsIndex = 10.0, num_segments: typing.SupportsInt | typing.SupportsIndex = 32, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_cross(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.SupportsFloat | typing.SupportsIndex = 7.070000171661377, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_cross_on(self, drawable_name: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.SupportsFloat | typing.SupportsIndex = 7.070000171661377, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_filled_rect(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_filled_rect_on(self, drawable_name: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_filled_triangle(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, height: typing.SupportsFloat | typing.SupportsIndex = 20.0, upsidedown: bool = True, centering: bool = False) -> int:
        ...
    def add_filled_triangle_on(self, drawable_name: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, height: typing.SupportsFloat | typing.SupportsIndex = 20.0, upsidedown: bool = True, centering: bool = False) -> int:
        ...
    def add_image(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], texture: pyridescence.glk.Texture, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., bg_color: typing.SupportsInt | typing.SupportsIndex = 0, border_color: typing.SupportsInt | typing.SupportsIndex = 0, border_thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_image_on(self, drawable_name: str, texture: pyridescence.glk.Texture, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., bg_color: typing.SupportsInt | typing.SupportsIndex = 0, border_color: typing.SupportsInt | typing.SupportsIndex = 0, border_thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0) -> int:
        ...
    def add_rect(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_rect_on(self, drawable: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, size: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ..., offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_text(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], text: str, fg_color: typing.SupportsInt | typing.SupportsIndex = 4294967295, bg_color: typing.SupportsInt | typing.SupportsIndex = 2147483648, offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_text_on(self, drawable_name: str, text: str, fg_color: typing.SupportsInt | typing.SupportsIndex = 4294967295, bg_color: typing.SupportsInt | typing.SupportsIndex = 2147483648, offset: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[2, 1]"] = ...) -> int:
        ...
    def add_triangle(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], color: typing.SupportsInt | typing.SupportsIndex = 4294967295, height: typing.SupportsFloat | typing.SupportsIndex = 20.0, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0, upsidedown: bool = True, centering: bool = False) -> int:
        ...
    def add_triangle_on(self, drawable: str, color: typing.SupportsInt | typing.SupportsIndex = 4294967295, height: typing.SupportsFloat | typing.SupportsIndex = 20.0, thickness: typing.SupportsFloat | typing.SupportsIndex = 1.0, upsidedown: bool = True, centering: bool = False) -> int:
        ...
    def clear(self) -> None:
        """
        Remove all drawings.
        """
    def create_callback(self) -> collections.abc.Callable[[], None]:
        """
        Create a UI callback function to draw the drawings. Register it with register_ui_callback().
        """
    def remove_drawing(self, drawing_id: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...
class Application:
    def close(self) -> None:
        ...
    def closed(self) -> bool:
        ...
    def disable_docking(self) -> None:
        ...
    def disable_vsync(self) -> None:
        ...
    def enable_docking(self) -> None:
        ...
    def enable_vsync(self) -> None:
        ...
    def framebuffer_size(self) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]:
        ...
    def fullscreen_window(self) -> None:
        ...
    def hide_window(self) -> None:
        ...
    def maximize_window(self) -> None:
        ...
    def ok(self) -> bool:
        ...
    def resize(self, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]) -> None:
        ...
    def set_title(self, title: str) -> None:
        ...
    def show_window(self) -> None:
        ...
    def spin(self) -> None:
        ...
    def spin_once(self) -> bool:
        ...
    def window_size(self) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[2, 1]"]:
        ...
class LightViewerContext:
    """
    Viewer context (the main viewer or a sub viewer).
    """
    @staticmethod
    def use_sensor_view_camera_control(*args, **kwargs) -> SensorViewCameraControl:
        """
        Use sensor-view camera control that follows a given sensor pose (set via SensorViewCameraControl.set_sensor_pose()).
        
        Args:
            T_sensor_camera: Initial transform from sensor frame to camera frame (4x4 matrix).
            smoothing_factor_trans: Smoothing factor for translation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
            smoothing_factor_rot: Smoothing factor for rotation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
        """
    def append_text(self, text: str) -> None:
        """
        Append debug text to the viewer. This method is thread-safe.
        """
    def backface_culling_enabled(self) -> bool:
        """
        Whether backface culling is enabled.
        """
    def clear(self) -> None:
        """
        Clear everything in the viewer (callbacks, filters, drawables, texts).
        """
    def clear_drawable_filters(self) -> None:
        """
        Clear all drawable filters.
        """
    def clear_drawables(self) -> None:
        """
        Clear all drawables.
        """
    def clear_partial_rendering(self) -> None:
        """
        Clear partial rendering buffer.
        """
    def clear_text(self) -> None:
        """
        Clear texts.
        """
    def color_buffer(self) -> pyridescence.glk.Texture:
        """
        Color buffer texture.
        """
    def depth_buffer(self) -> pyridescence.glk.Texture:
        """
        Depth buffer texture.
        """
    def disable_backface_culling(self) -> None:
        """
        Disable backface culling.
        """
    def disable_partial_rendering(self) -> None:
        """
        Disable partial rendering.
        """
    def disable_xy_grid(self) -> None:
        """
        Disable XY grid drawing.
        """
    def enable_backface_culling(self) -> None:
        """
        Enable backface culling. Fragments with back-facing normals will not be rendered.
        """
    def enable_info_buffer(self) -> None:
        """
        Enable info buffer (required for pick_info()).
        """
    def enable_normal_buffer(self) -> None:
        """
        Enable normal buffer.
        """
    def enable_partial_rendering(self, clear_thresh: typing.SupportsFloat | typing.SupportsIndex = 1e-06) -> None:
        """
        Enable partial rendering.
        
        Args:
            clear_thresh: Threshold for clearing the partial rendering buffer.
        """
    def enable_xy_grid(self) -> None:
        """
        Enable XY grid drawing.
        """
    def find_drawable(self, name: str) -> tuple[ShaderSetting, pyridescence.glk.Drawable]:
        """
        Find a drawable by name. Returns a (shader_setting, drawable) pair if found, otherwise (None, None).
        """
    def get_camera_control(self) -> CameraControl:
        """
        Get camera control (view matrix control).
        """
    def get_drawables(self) -> dict[str, tuple[ShaderSetting, pyridescence.glk.Drawable]]:
        """
        Get all drawables as a dict of name -> (shader_setting, drawable). This method may be removed in the future.
        """
    def get_projection_control(self) -> ProjectionControl:
        """
        Get projection control (projection matrix control).
        """
    def get_screen_effect(self) -> pyridescence.glk.ScreenEffect:
        """
        Get screen effect.
        """
    def hide(self) -> None:
        """
        Hide the sub viewer window.
        """
    def info_buffer(self) -> pyridescence.glk.Texture:
        """
        Info buffer texture. enable_info_buffer() must be called beforehand.
        """
    def load_camera_settings(self, path: str) -> bool:
        """
        Load camera settings from a file.
        """
    def lookat(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
        Make the camera look at a point.
        """
    def normal_buffer(self) -> pyridescence.glk.Texture:
        """
        Normal buffer texture. enable_normal_buffer() must be called beforehand.
        """
    def pick_depth(self, p: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], window: typing.SupportsInt | typing.SupportsIndex = 2) -> float:
        """
        Pick depth from the depth buffer.
        
        Args:
            p: Pixel coordinates.
            window: Window size for finding a valid pixel neighboring p.
        
        Returns:
            Depth value. Depth > 1.0 indicates background.
        """
    def pick_info(self, p: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], window: typing.SupportsInt | typing.SupportsIndex = 2) -> typing.Annotated[numpy.typing.NDArray[numpy.int32], "[4, 1]"]:
        """
        Pick info from the info buffer. enable_info_buffer() must be called beforehand.
        
        Args:
            p: Pixel coordinates.
            window: Window size for finding a valid pixel neighboring p.
        
        Returns:
            Info vector (4 ints).
        """
    def pick_point(self, button: typing.SupportsInt | typing.SupportsIndex = 0, window: typing.SupportsInt | typing.SupportsIndex = 2) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"] | None:
        """
        Pick a 3D point under the mouse cursor when the mouse button is clicked. Call this in a UI callback.
        
        Args:
            button: Mouse button index (0: left, 1: right, 2: middle).
            window: Window size for finding a valid pixel neighboring the mouse position.
        
        Returns:
            3D point in the world coordinate, or None if the button is not clicked or no valid depth is found.
        """
    def register_drawable_filter(self, filter_name: str, filter: collections.abc.Callable[[str], bool]) -> None:
        """
        Register a drawable filter. Drawables for which filter(name) returns False are not rendered.
        
        Args:
            filter_name: Filter name.
            filter: Filter function that takes a drawable name and returns a bool. If None, the filter with the given name is removed.
        """
    def register_ui_callback(self, callback_name: str, callback: collections.abc.Callable[[], None]) -> None:
        """
        Register a UI callback function that is called every frame to draw ImGui UI.
        
        Args:
            callback_name: Callback name. If a callback with the same name exists, it is replaced.
            callback: Callback function. If None, the callback with the given name is removed.
        """
    def remove_drawable(self, pattern: str, regex: bool = False) -> None:
        """
        Remove drawables.
        
        Args:
            pattern: Drawable name, or a regular expression if regex is True.
            regex: If True, all drawables whose names match pattern are removed.
        """
    def remove_drawable_filter(self, filter_name: str) -> None:
        """
        Remove a drawable filter.
        """
    def remove_ui_callback(self, callback_name: str) -> None:
        """
        Remove a UI callback function.
        """
    def reset_center(self) -> None:
        """
        Reset camera center to the center of the screen.
        """
    def save_camera_settings(self, path: str) -> bool:
        """
        Save camera settings to a file.
        """
    def save_color_buffer(self, path: str) -> bool:
        """
        Save color buffer to an image file (PNG).
        """
    def save_depth_buffer(self, path: str, real_scale: bool = True) -> bool:
        """
        Save depth buffer to a file (PNG). If real_scale is True, depth values are scaled to real-world values.
        """
    def set_backface_culling_range(self, min_depth: typing.SupportsFloat | typing.SupportsIndex, max_depth: typing.SupportsFloat | typing.SupportsIndex) -> None:
        """
        Set the [min, max] range of normal.z for backface culling. Fragments with normal.z outside the range are culled when backface culling is enabled.
        """
    def set_bg_texture(self, bg_texture: pyridescence.glk.Texture) -> None:
        """
        Set background texture.
        """
    def set_camera_control(self, camera_control: CameraControl) -> None:
        """
        Set camera control (view matrix control).
        """
    def set_clear_color(self, color: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        """
        Set the background clear color (RGBA).
        """
    def set_colormap(self, colormap: pyridescence.glk.COLORMAP) -> None:
        """
        Set colormap used for RAINBOW and VERTEX_COLORMAP modes.
        """
    def set_draw_xy_grid(self, draw_xy_grid: bool) -> None:
        """
        Set enable/disable XY grid drawing.
        """
    def set_point_shape(self, point_size: typing.SupportsFloat | typing.SupportsIndex = 1.0, metric: bool = True, circle: bool = True) -> None:
        """
        Set point shape properties.
        
        Args:
            point_size: Point size.
            metric: If True, point size is in the metric unit [m], otherwise in pixel unit.
            circle: If True, points are rendered as circles, otherwise as squares.
        """
    def set_pos(self, pos: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], cond: typing.SupportsInt | typing.SupportsIndex = 4, flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Set the position of the sub viewer window.
        """
    def set_projection_control(self, projection_control: ProjectionControl) -> None:
        """
        Set projection control (projection matrix control).
        """
    def set_screen_effect(self, effect: pyridescence.glk.ScreenEffect) -> None:
        """
        Set screen effect.
        """
    def set_size(self, size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"]) -> None:
        """
        Set the canvas size.
        """
    def shader_setting(self) -> ShaderSetting:
        """
        Global shader setting applied to all drawables.
        """
    def show(self) -> None:
        """
        Show the sub viewer window.
        """
    def unproject(self, p: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"], depth: typing.SupportsFloat | typing.SupportsIndex) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
        Unproject a pixel to a 3D point.
        
        Args:
            p: Pixel coordinates.
            depth: Depth value (e.g., obtained by pick_depth()).
        
        Returns:
            3D point in the world coordinate.
        """
    def update_cone(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a cone primitive.
        """
    def update_coord(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a coordinate axes primitive.
        """
    def update_cube(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a cube primitive.
        """
    def update_drawable(self, name: str, drawable: typing.Any, shader_setting: ShaderSetting = ...) -> None:
        """
        Register or update a drawable.
        
        Args:
            name: Drawable name. If a drawable with the same name exists, it is replaced.
            drawable: Drawable.
            shader_setting: Shader setting. It is copied, and later changes to it are not reflected.
        """
    def update_frustum(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a frustum primitive.
        """
    def update_icosahedron(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update an icosahedron primitive.
        """
    @typing.overload
    def update_normal_dists(self, name: str, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], covs: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]], sigma: typing.SupportsFloat | typing.SupportsIndex, shader_setting: ShaderSetting) -> None:
        """
        Register or update a normal distribution set (rendered as ellipsoids).
        
        Args:
            name: Drawable name.
            points: Point means.
            covs: Point covariances. Must have the same size as points.
            sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_normal_dists(self, name: str, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]], covs: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]], sigma: typing.SupportsFloat | typing.SupportsIndex, shader_setting: ShaderSetting) -> None:
        """
        Register or update a normal distribution set (rendered as ellipsoids).
        
        Args:
            name: Drawable name.
            points: Point means.
            covs: Point covariances. Must have the same size as points.
            sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
            shader_setting: Shader setting.
        """
    def update_points(self, name: str, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], shader_setting: ShaderSetting) -> pyridescence.glk.PointCloudBuffer:
        """
        Register or update a point cloud buffer.
        
        Args:
            name: Drawable name.
            points: Point positions (N x 3 or N x 4 array).
            shader_setting: Shader setting.
        
        Returns:
            Created PointCloudBuffer, or None if points has an invalid shape.
        """
    def update_sphere(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a sphere primitive.
        """
    @typing.overload
    def update_thin_lines(self, name: str, vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"] = ..., indices: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex] = [], line_strip: bool = False, shader_setting: ShaderSetting = ...) -> pyridescence.glk.ThinLines:
        """
        Register or update a thin lines drawable.
        
        Args:
            name: Drawable name.
            vertices: Vertex positions (N x 3 or N x 4 array).
            colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
            indices: Vertex indices. If empty, non-indexed drawing is used.
            line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_thin_lines(self, name: str, vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"] = ..., indices: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex] = [], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0, shader_setting: ShaderSetting = ...) -> pyridescence.glk.ThinLines:
        """
        Register or update a thin lines drawable.
        
        Args:
            name: Drawable name.
            vertices: Vertex positions (N x 3 or N x 4 array).
            colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
            indices: Vertex indices. If empty, non-indexed drawing is used.
            line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
            line_width: Width of the lines in pixels.
            shader_setting: Shader setting.
        """
    def update_wire_cone(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe cone primitive.
        """
    def update_wire_cube(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe cube primitive.
        """
    def update_wire_frustum(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe frustum primitive.
        """
    def update_wire_icosahedron(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe icosahedron primitive.
        """
    def update_wire_sphere(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe sphere primitive.
        """
    def use_arcball_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = -1.0471975511965976) -> ArcBallCameraControl:
        """
        Use arcball camera control.
        """
    def use_fps_camera_control(self, fovy_deg: typing.SupportsFloat | typing.SupportsIndex = 60.0) -> FPSCameraControl:
        """
        Use FPS game-like camera control.
        """
    def use_orbit_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = -1.0471975511965976) -> OrbitCameraControlXY:
        """
        Use orbit camera control (XY plane) useful for systems in which XYZ = front-left-up like LiDAR odometry.
        """
    def use_orbit_camera_control_xz(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = 0.0) -> OrbitCameraControlXZ:
        """
        Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.
        """
    def use_orthographic_projection_control(self, width: typing.SupportsFloat | typing.SupportsIndex = 10.0, near: typing.SupportsFloat | typing.SupportsIndex = 1.0, far: typing.SupportsFloat | typing.SupportsIndex = 1000.0) -> BasicProjectionControl:
        """
        Use orthographic projection control.
        """
    def use_perspective_projection_control(self, fovy_deg: typing.SupportsFloat | typing.SupportsIndex = 30.0, near: typing.SupportsFloat | typing.SupportsIndex = 1.0, far: typing.SupportsFloat | typing.SupportsIndex = 1000.0) -> BasicProjectionControl:
        """
        Use perspective projection control.
        """
    def use_topdown_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0) -> TopDownCameraControl:
        """
        Use top-down camera control useful for 2D SLAM.
        """
class LightViewer(Application, LightViewerContext):
    """
    Light viewer. The main visualization class in Iridescence. Use guik.viewer() or LightViewer.instance() to get the singleton instance.
    """
    @staticmethod
    def instance(size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"] = ..., background: bool = False, title: str = 'screen') -> LightViewer:
        """
        Get the singleton instance of LightViewer. The viewer instance is created upon the first call.
        
        Args:
            size: Initial canvas size. Ignored if the viewer has already been created.
            background: If True, the viewer window is created in the background (invisible).
            title: Window title.
        """
    def clear_images(self) -> None:
        """
        Clear all images.
        """
    def clear_plots(self, clear_settings: bool = True) -> None:
        """
        Clear all plots. If clear_settings is True, plot settings are cleared as well.
        """
    def fit_all_plots(self) -> None:
        """
        Fit all plots to their data ranges.
        """
    def fit_plot(self, plot_name: str) -> None:
        """
        Fit the plot view to the data range.
        """
    def invoke(self, func: collections.abc.Callable[[], None]) -> None:
        """
        Invoke a task in the visualization thread.
        """
    def invoke_after_rendering(self, func: collections.abc.Callable[[], None]) -> None:
        """
        Invoke a task after rendering in the visualization thread.
        """
    def invoke_once(self, label: str, func: collections.abc.Callable[[], None]) -> None:
        """
        Invoke a labeled task only once in the life time of the viewer.
        """
    def link_plot_axes(self, plot_name: str, link_id: typing.SupportsInt | typing.SupportsIndex, axes: typing.SupportsInt | typing.SupportsIndex = -1) -> None:
        """
        Link multiple axes of a plot to a link ID. Axes with the same link ID share their ranges.
        
        Args:
            plot_name: Name of the plot.
            link_id: Link ID.
            axes: Bitwise OR of axes to be linked ((1 << ImAxis_X1) | (1 << ImAxis_Y1) ...). If -1, all axes are linked.
        """
    def link_plot_axis(self, plot_name: str, link_id: typing.SupportsInt | typing.SupportsIndex, axis: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Link an axis of a plot to a link ID. Axes with the same link ID share their ranges.
        
        Args:
            plot_name: Name of the plot.
            link_id: Link ID.
            axis: Axis to be linked (ImAxis: X1 = 0, X2 = 1, X3 = 2, Y1 = 3, Y2 = 4, Y3 = 5).
        """
    def remove_image(self, name: str) -> None:
        """
        Remove an image by name.
        """
    def remove_plot(self, plot_name: str, label: str = '') -> None:
        """
        Remove a plot by name.
        
        Args:
            plot_name: Name of the plot to be removed.
            label: If non-empty, remove only the plot with the specified label in the plot group.
        """
    def setup_legend(self, plot_name: str, loc: typing.SupportsInt | typing.SupportsIndex, flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Setup the legend for a plot.
        
        Args:
            plot_name: Name of the plot.
            loc: Location of the legend (ImPlotLocation).
            flags: Flags for the legend (ImPlotLegendFlags).
        """
    def setup_plot(self, plot_name: str, width: typing.SupportsInt | typing.SupportsIndex, height: typing.SupportsInt | typing.SupportsIndex, plot_flags: typing.SupportsInt | typing.SupportsIndex = 0, x_flags: typing.SupportsInt | typing.SupportsIndex = 0, y_flags: typing.SupportsInt | typing.SupportsIndex = 0, order: typing.SupportsInt | typing.SupportsIndex = -1) -> None:
        """
        Setup a plot.
        
        Args:
            plot_name: Name of the plot.
            width: Width of the plot [pixels].
            height: Height of the plot [pixels].
            plot_flags: Flags for the plot (ImPlotFlags).
            x_flags: Flags for the x-axis (ImPlotAxisFlags).
            y_flags: Flags for the y-axis (ImPlotAxisFlags).
            order: Display order of the plot. Plots with smaller order values are displayed first. If -1, the plot is added to the end of the display order.
        """
    def setup_plot_group_order(self, group_name: str, order: typing.SupportsInt | typing.SupportsIndex) -> None:
        """
        Setup the plot group order.
        
        Args:
            group_name: Name of the plot group.
            order: Display order of the plot group. Plot groups with smaller order values are displayed first.
        """
    def spin_until_click(self) -> bool:
        """
        Spin the viewer until a mouse click event is received.
        """
    def sub_viewer(self, name: str, size: tuple[typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex] = (-1, -1)) -> LightViewerContext:
        """
        Create or get a sub viewer context.
        
        Args:
            name: Name of the sub viewer context.
            size: Canvas size of the sub viewer. If (-1, -1), the default size is used.
        """
    def toggle_spin_once(self) -> bool:
        """
        Spin the viewer until a toggle flag is set to true.
        """
    def update_image(self, name: str, image: pyridescence.glk.Texture, scale: typing.SupportsFloat | typing.SupportsIndex = -1.0, order: typing.SupportsInt | typing.SupportsIndex = -1) -> None:
        """
        Register or update an image.
        
        Args:
            name: Name of the image.
            image: Image texture.
            scale: Scale factor for displaying the image. If negative, auto scale is used.
            order: Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
        """
    def update_plot_histogram(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], x_bins: typing.SupportsInt | typing.SupportsIndex = -2, y_bins: typing.SupportsInt | typing.SupportsIndex = -2, x_range: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"] = ..., y_range: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"] = ..., histogram_flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Update a histogram plot. If ys is empty, a 1D histogram of xs is plotted. Otherwise, a 2D histogram of (xs, ys) is plotted.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: Data to be histogrammed (X data for 2D histogram).
            ys: Y data for 2D histogram.
            x_bins: Number of bins along X, or an ImPlotBin method (negative value, e.g., -2 = Sturges).
            y_bins: Number of bins along Y, or an ImPlotBin method (negative value, e.g., -2 = Sturges).
            x_range: Data range along X. If (0, 0), the min/max of the data is used.
            y_range: Data range along Y. If (0, 0), the min/max of the data is used.
            histogram_flags: Flags for the histogram plot (ImPlotHistogramFlags).
        """
    def update_plot_line(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], line_flags: typing.SupportsInt | typing.SupportsIndex = 0, max_num_data: typing.SupportsInt | typing.SupportsIndex = 1048576) -> None:
        """
        Update a line plot.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
            ys: Y data.
            line_flags: Flags for the line plot (ImPlotLineFlags).
            max_num_data: Maximum number of data points to be displayed.
        """
    def update_plot_scatter(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], scatter_flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Update a scatter plot.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
            ys: Y data.
            scatter_flags: Flags for the scatter plot (ImPlotScatterFlags).
        """
    def update_plot_stairs(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], stairs_flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Update a stairs plot.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
            ys: Y data. Must have the same length as xs.
            stairs_flags: Flags for the stairs plot (ImPlotStairsFlags).
        """
class AsyncLightViewerContext:
    """
    Async interface to manipulate LightViewerContext.
    All OpenGL operations are called in the viewer thread through invoke().
    Note that async operations come at a cost of data copy.
    """
    @staticmethod
    def use_sensor_view_camera_control(*args, **kwargs) -> None:
        """
        Use TPS-like sensor-view camera control.
        
        Args:
            T_sensor_camera: Initial transform from sensor frame to camera frame (4x4 matrix).
            smoothing_factor_trans: Smoothing factor for translation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
            smoothing_factor_rot: Smoothing factor for rotation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
        """
    def append_text(self, text: str) -> None:
        """
        Append debug text to the viewer. This method is thread-safe.
        """
    def clear(self) -> None:
        """
        Clear everything in the viewer (callbacks, filters, drawables, texts).
        """
    def clear_drawables(self) -> None:
        """
        Clear all drawables.
        """
    def clear_text(self) -> None:
        """
        Clear texts.
        """
    def disable_xy_grid(self) -> None:
        """
        Disable XY grid drawing.
        """
    def enable_xy_grid(self) -> None:
        """
        Enable XY grid drawing.
        """
    def load_camera_settings(self, path: str) -> None:
        """
        Load camera settings from a file.
        """
    def lookat(self, pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
        Make the camera look at a point.
        """
    def register_ui_callback(self, callback_name: str, callback: collections.abc.Callable[[], None]) -> None:
        """
        Register a UI callback function that is called every frame in the viewer thread to draw ImGui UI.
        
        Args:
            callback_name: Callback name. If a callback with the same name exists, it is replaced.
            callback: Callback function. If None, the callback with the given name is removed.
        """
    def remove_drawable(self, pattern: str, regex: bool = False) -> None:
        """
        Remove drawables.
        
        Args:
            pattern: Drawable name, or a regular expression if regex is True.
            regex: If True, all drawables whose names match pattern are removed.
        """
    def remove_ui_callback(self, callback_name: str) -> None:
        """
        Remove a UI callback function.
        """
    def reset_center(self) -> None:
        """
        Reset camera center to the center of the screen.
        """
    def save_camera_settings(self, path: str) -> None:
        """
        Save camera settings to a file.
        """
    def save_color_buffer(self, path: str) -> None:
        """
        Save color buffer to an image file (PNG).
        """
    def save_depth_buffer(self, path: str, real_scale: bool = True) -> None:
        """
        Save depth buffer to a file (PNG). If real_scale is True, depth values are scaled to real-world values.
        """
    def set_colormap(self, colormap: pyridescence.glk.COLORMAP) -> None:
        """
        Set colormap used for RAINBOW and VERTEX_COLORMAP modes.
        """
    def set_draw_xy_grid(self, draw_xy_grid: bool) -> None:
        """
        Set enable/disable XY grid drawing.
        """
    def set_point_shape(self, point_size: typing.SupportsFloat | typing.SupportsIndex = 1.0, metric: bool = True, circle: bool = True) -> None:
        """
        Set point shape properties.
        
        Args:
            point_size: Point size.
            metric: If True, point size is in the metric unit [m], otherwise in pixel unit.
            circle: If True, points are rendered as circles, otherwise as squares.
        """
    def update_cone(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a cone primitive.
        """
    def update_coord(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a coordinate axes primitive.
        """
    def update_cube(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a cube primitive.
        """
    def update_drawable_setting(self, name: str, setting: ShaderSetting) -> None:
        """
        Update a drawable's shader setting.
        """
    def update_frustum(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a frustum primitive.
        """
    def update_icosahedron(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update an icosahedron primitive.
        """
    @typing.overload
    def update_normal_dists(self, name: str, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]], covs: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]], sigma: typing.SupportsFloat | typing.SupportsIndex, shader_setting: ShaderSetting) -> None:
        """
        Register or update a normal distribution set (rendered as ellipsoids).
        
        Args:
            name: Drawable name.
            points: Point means.
            covs: Point covariances. Must have the same size as points.
            sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_normal_dists(self, name: str, points: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]], covs: collections.abc.Sequence[typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 4]"]], sigma: typing.SupportsFloat | typing.SupportsIndex, shader_setting: ShaderSetting) -> None:
        """
        Register or update a normal distribution set (rendered as ellipsoids).
        
        Args:
            name: Drawable name.
            points: Point means.
            covs: Point covariances. Must have the same size as points.
            sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_points(self, name: str, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], shader_setting: ShaderSetting) -> None:
        """
        Register or update a point cloud buffer.
        
        Args:
            name: Drawable name.
            points: Point positions (N x 3 or N x 4 array).
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_points(self, name: str, points: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], shader_setting: ShaderSetting) -> None:
        """
        Register or update a point cloud buffer with colors.
        
        Args:
            name: Drawable name.
            points: Point positions (N x 3 or N x 4 array).
            colors: Point colors (N x 3 or N x 4 array).
            shader_setting: Shader setting.
        """
    def update_sphere(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a sphere primitive.
        """
    @typing.overload
    def update_thin_lines(self, name: str, vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"] = ..., indices: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex] = [], line_strip: bool = False, shader_setting: ShaderSetting = ...) -> None:
        """
        Register or update a thin lines drawable.
        
        Args:
            name: Drawable name.
            vertices: Vertex positions (N x 3 or N x 4 array).
            colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
            indices: Vertex indices. If empty, non-indexed drawing is used.
            line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
            shader_setting: Shader setting.
        """
    @typing.overload
    def update_thin_lines(self, name: str, vertices: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"], colors: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, n]"] = ..., indices: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex] = [], line_strip: bool = False, line_width: typing.SupportsFloat | typing.SupportsIndex = 1.0, shader_setting: ShaderSetting = ...) -> None:
        """
        Register or update a thin lines drawable.
        
        Args:
            name: Drawable name.
            vertices: Vertex positions (N x 3 or N x 4 array).
            colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
            indices: Vertex indices. If empty, non-indexed drawing is used.
            line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
            line_width: Width of the lines in pixels.
            shader_setting: Shader setting.
        """
    def update_wire_cone(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe cone primitive.
        """
    def update_wire_cube(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe cube primitive.
        """
    def update_wire_frustum(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe frustum primitive.
        """
    def update_wire_icosahedron(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe icosahedron primitive.
        """
    def update_wire_sphere(self, name: str, shader_setting: ShaderSetting) -> None:
        """
        Register or update a wireframe sphere primitive.
        """
    def use_arcball_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = -1.0471975511965976) -> None:
        """
        Use arcball camera control.
        """
    def use_fps_camera_control(self, fovy_deg: typing.SupportsFloat | typing.SupportsIndex = 60.0) -> None:
        """
        Use FPS game-like camera control.
        """
    def use_orbit_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = -1.0471975511965976) -> None:
        """
        Use orbit camera control (XY plane) useful for systems in which XYZ = front-left-up like LiDAR odometry.
        """
    def use_orbit_camera_control_xz(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0, phi: typing.SupportsFloat | typing.SupportsIndex = 0.0) -> None:
        """
        Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.
        """
    def use_orthographic_projection_control(self, width: typing.SupportsFloat | typing.SupportsIndex = 10.0, near: typing.SupportsFloat | typing.SupportsIndex = 1.0, far: typing.SupportsFloat | typing.SupportsIndex = 1000.0) -> None:
        """
        Use orthographic projection control.
        """
    def use_perspective_projection_control(self, fovy_deg: typing.SupportsFloat | typing.SupportsIndex = 30.0, near: typing.SupportsFloat | typing.SupportsIndex = 1.0, far: typing.SupportsFloat | typing.SupportsIndex = 1000.0) -> None:
        """
        Use perspective projection control.
        """
    def use_topdown_camera_control(self, distance: typing.SupportsFloat | typing.SupportsIndex = 80.0, theta: typing.SupportsFloat | typing.SupportsIndex = 0.0) -> None:
        """
        Use top-down camera control useful for 2D SLAM.
        """
class AsyncLightViewer(AsyncLightViewerContext):
    @staticmethod
    def destroy() -> None:
        """
        Destroy the viewer in the background thread.
        """
    @staticmethod
    def instance(size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"] = ..., background: bool = False, title: str = 'screen') -> AsyncLightViewer:
        """
        Get the singleton instance of AsyncLightViewer. The viewer is created and run in a background thread upon the first call.
        
        Args:
            size: Initial canvas size. Ignored if the viewer has already been created.
            background: If True, the viewer window is created in the background (invisible).
            title: Window title.
        """
    @staticmethod
    def toggle_wait() -> None:
        """
        Wait while a toggle checkbox is checked.
        """
    @staticmethod
    def wait() -> None:
        """
        Wait for the viewer to be closed.
        """
    @staticmethod
    def wait_until_click() -> None:
        """
        Wait for a break button to be clicked.
        """
    def async_sub_viewer(self, name: str, size: tuple[typing.SupportsInt | typing.SupportsIndex, typing.SupportsInt | typing.SupportsIndex] = (-1, -1)) -> AsyncLightViewerContext:
        """
        Create or get an async sub viewer context.
        
        Args:
            name: Name of the sub viewer context.
            size: Canvas size of the sub viewer. If (-1, -1), the default size is used.
        """
    def clear_images(self) -> None:
        """
        Clear all images.
        """
    def fit_all_plots(self) -> None:
        """
        Fit all plots to their data ranges.
        """
    def fit_plot(self, plot_name: str) -> None:
        """
        Fit the plot view to the data range.
        """
    def remove_async_sub_viewer(self, name: str) -> None:
        """
        Remove an async sub viewer context by name.
        """
    def remove_image(self, name: str) -> None:
        """
        Remove an image by name.
        """
    def update_image(self, name: str, width: typing.SupportsInt | typing.SupportsIndex, height: typing.SupportsInt | typing.SupportsIndex, rgba_bytes: collections.abc.Sequence[typing.SupportsInt | typing.SupportsIndex], scale: typing.SupportsFloat | typing.SupportsIndex = -1.0, order: typing.SupportsInt | typing.SupportsIndex = -1) -> None:
        """
        Register or update an image.
        
        Args:
            name: Name of the image.
            width: Image width.
            height: Image height.
            rgba_bytes: Image data in RGBA format (width * height * 4 bytes).
            scale: Scale factor for displaying the image. If negative, auto scale is used.
            order: Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
        """
    def update_plot_histogram(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], x_bins: typing.SupportsInt | typing.SupportsIndex = -2, y_bins: typing.SupportsInt | typing.SupportsIndex = -2, x_range: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"] = ..., y_range: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"] = ..., histogram_flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Update a histogram plot. If ys is empty, a 1D histogram of xs is plotted. Otherwise, a 2D histogram of (xs, ys) is plotted.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: Data to be histogrammed (X data for 2D histogram).
            ys: Y data for 2D histogram.
            x_bins: Number of bins along X, or an ImPlotBin method (negative value, e.g., -2 = Sturges).
            y_bins: Number of bins along Y, or an ImPlotBin method (negative value, e.g., -2 = Sturges).
            x_range: Data range along X. If (0, 0), the min/max of the data is used.
            y_range: Data range along Y. If (0, 0), the min/max of the data is used.
            histogram_flags: Flags for the histogram plot (ImPlotHistogramFlags).
        """
    def update_plot_line(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], line_flags: typing.SupportsInt | typing.SupportsIndex = 0, max_num_data: typing.SupportsInt | typing.SupportsIndex = 1048576) -> None:
        """
        Update a line plot.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
            ys: Y data.
            line_flags: Flags for the line plot (ImPlotLineFlags).
            max_num_data: Maximum number of data points to be displayed.
        """
    def update_plot_scatter(self, plot_name: str, label: str, xs: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex], ys: collections.abc.Sequence[typing.SupportsFloat | typing.SupportsIndex] = [], scatter_flags: typing.SupportsInt | typing.SupportsIndex = 0) -> None:
        """
        Update a scatter plot.
        
        Args:
            plot_name: Name of the plot.
            label: Label.
            xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
            ys: Y data.
            scatter_flags: Flags for the scatter plot (ImPlotScatterFlags).
        """
class RecentFiles:
    """
    History of recently opened files, persisted per tag.
    """
    def __init__(self, tag: str) -> None:
        ...
    def clear(self) -> None:
        ...
    def empty(self) -> bool:
        ...
    def filename(self, i: typing.SupportsInt | typing.SupportsIndex) -> str:
        ...
    def fullpath(self, i: typing.SupportsInt | typing.SupportsIndex) -> str:
        ...
    def most_recent(self) -> str:
        ...
    def push(self, filename: str) -> None:
        ...
    def push_all(self, filenames: collections.abc.Sequence[str]) -> None:
        ...
    def size(self) -> int:
        ...
def anon() -> str:
    """
    Generate a unique name.
    """
def async_destroy() -> None:
    """
    Destroy the async viewer
    """
def async_running() -> bool:
    """
    Check if the async viewer is currently running
    """
def async_viewer(size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"] = ..., background: bool = False, title: str = 'screen') -> AsyncLightViewer:
    """
    Get the singleton AsyncLightViewer instance. The viewer is created and run in a background thread upon the first call.
    
    Args:
        size: Initial canvas size. Ignored if the viewer has already been created.
        background: If True, the viewer window is created in the background (invisible).
        title: Window title.
    """
def async_wait() -> None:
    """
    Wait for the async viewer to be closed.
    """
def destroy() -> None:
    """
    Destroy the viewer
    """
def running() -> bool:
    """
    Check if the viewer is currently running
    """
def viewer(size: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[2, 1]"] = ..., background: bool = False, title: str = 'screen') -> LightViewer:
    """
    Get the singleton LightViewer instance. The viewer instance is created upon the first call.
    
    Args:
        size: Initial canvas size. Ignored if the viewer has already been created.
        background: If True, the viewer window is created in the background (invisible).
        title: Window title.
    """
CIRCLE: PointShapeMode
METRIC: PointScaleMode
RECTANGLE: PointShapeMode
SCREENSPACE: PointScaleMode
