#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <filesystem>

#include <glk/path.hpp>
#include <glk/texture.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/drawable_container.hpp>
#include <guik/recent_files.hpp>
#include <guik/model_control.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/arcball_camera_control.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/camera/fps_camera_control.hpp>
#include <guik/camera/sensor_view_camera_control.hpp>
#include <guik/camera/basic_projection_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer.hpp>

#include "py_drawable.hpp"

namespace py = pybind11;

static bool is_first = true;

template <typename Color>
void define_flat_color(py::module_& guik_, const std::string& name) {
  py::class_<Color, guik::FlatColor, std::shared_ptr<Color>>(guik_, name.c_str())
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>(), py::arg("model_matrix"))
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new Color(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());
}

guik::LightViewer* instance(const Eigen::Vector2i& size, bool background, const std::string& title) {
  if (is_first) {
    py::gil_scoped_acquire acquire;
    py::object pyridescence = py::module::import("pyridescence_data");
    std::filesystem::path path(pyridescence.attr("__file__").cast<std::string>());

    glk::set_data_path(path.parent_path().string());
    is_first = false;
  }

  static guik::LightViewer* inst = guik::LightViewer::instance(size, background, title);
  return inst;
}

guik::AsyncLightViewer* async_instance(const Eigen::Vector2i& size, bool background, const std::string& title) {
  if (is_first) {
    py::gil_scoped_acquire acquire;
    py::object pyridescence = py::module::import("pyridescence_data");
    std::filesystem::path path(pyridescence.attr("__file__").cast<std::string>());

    glk::set_data_path(path.parent_path().string());
    is_first = false;
  }

  return guik::AsyncLightViewer::instance(size, background, title);
}

void define_guik(py::module_& m) {
  py::module_ guik_ = m.def_submodule("guik", "");

  // enums
  // guik::ColorMode
  py::enum_<guik::PointShapeMode::MODE>(guik_, "PointShapeMode", R"doc(Point shape mode.

RECTANGLE: Square/rectangle shape
CIRCLE: Circle shape
)doc")
    .value("RECTANGLE", guik::PointShapeMode::MODE::RECTANGLE)
    .value("CIRCLE", guik::PointShapeMode::MODE::CIRCLE)
    .export_values();

  // guik::PointScaleMode
  py::enum_<guik::PointScaleMode::MODE>(guik_, "PointScaleMode", R"doc(Point size scale mode.

SCREENSPACE: Screen space size
METRIC: Metric size (in meters)
)doc")
    .value("SCREENSPACE", guik::PointScaleMode::MODE::SCREENSPACE)
    .value("METRIC", guik::PointScaleMode::MODE::METRIC)
    .export_values();

  // classess
  // guik::ShaderSetting
  py::class_<guik::ShaderSetting, std::shared_ptr<guik::ShaderSetting>>(
    guik_,
    "ShaderSetting",
    R"doc(Shader setting that holds rendering settings (color mode, model matrix, point shape, etc.).

Setter methods return the shader setting itself so that they can be chained (e.g., guik.FlatRed().set_point_size(2.0).translate(1.0, 0.0, 0.0)).
)doc")
    .def(
      "add",
      &guik::ShaderSetting::add<float>,
      py::arg("name"),
      py::arg("value"),
      R"doc(Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.

Args:
    name: Parameter name (uniform variable name in the shader).
    value: Parameter value.
)doc")
    .def(
      "add",
      &guik::ShaderSetting::add<Eigen::Vector2f>,
      py::arg("name"),
      py::arg("value"),
      R"doc(Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.

Args:
    name: Parameter name (uniform variable name in the shader).
    value: Parameter value.
)doc")
    .def(
      "add",
      &guik::ShaderSetting::add<Eigen::Matrix4f>,
      py::arg("name"),
      py::arg("value"),
      R"doc(Add a new parameter to the shader setting. If a parameter with the same name exists, it is overwritten.

Args:
    name: Parameter name (uniform variable name in the shader).
    value: Parameter value.
)doc")
    .def(
      "addi",
      &guik::ShaderSetting::add<Eigen::Vector4i>,
      py::arg("name"),
      py::arg("value"),
      R"doc(Add a new integer vector (ivec4) parameter to the shader setting. If a parameter with the same name exists, it is overwritten.

Args:
    name: Parameter name (uniform variable name in the shader).
    value: Parameter value.
)doc")
    .def("static_object", &guik::ShaderSetting::static_object, py::return_value_policy::reference_internal, "Set static object flag.")
    .def("dynamic_object", &guik::ShaderSetting::dynamic_object, py::return_value_policy::reference_internal, "Set dynamic object flag.")
    .def("material_color", &guik::ShaderSetting::material_color, "Get material color (for FLAT_COLOR).")
    .def(
      "set_color",
      [&](guik::ShaderSetting& self, float r, float g, float b, float a) { return self.set_color(Eigen::Vector4f(r, g, b, a)); },
      py::arg("r"),
      py::arg("g"),
      py::arg("b"),
      py::arg("a") = 1.0,
      py::return_value_policy::reference_internal,
      "Set material color (for FLAT_COLOR). If a < 0.999, the object is rendered as transparent.")
    .def(
      "set_color",
      [&](guik::ShaderSetting& self, const Eigen::Vector4f& color) { return self.set_color(color); },
      py::arg("color"),
      py::return_value_policy::reference_internal,
      "Set material color (RGBA, for FLAT_COLOR). If alpha < 0.999, the object is rendered as transparent.")
    .def(
      "set_alpha",
      &guik::ShaderSetting::set_alpha,
      py::arg("alpha"),
      py::return_value_policy::reference_internal,
      "Set alpha value of the material color. If alpha < 0.999, the object is rendered as transparent.")
    .def("make_transparent", &guik::ShaderSetting::make_transparent, py::return_value_policy::reference_internal, "Make the object transparent (alpha blending).")
    .def("point_scale", &guik::ShaderSetting::point_scale, "Get point size scale factor.")
    .def("set_point_scale", &guik::ShaderSetting::set_point_scale, py::arg("scale"), py::return_value_policy::reference_internal, "Set point size scale factor.")
    .def("set_point_size", &guik::ShaderSetting::set_point_size, py::arg("size"), py::return_value_policy::reference_internal, "Set point size.")
    .def("set_point_size_offset", &guik::ShaderSetting::set_point_size_offset, py::arg("offset"), py::return_value_policy::reference_internal, "Set point size offset.")
    .def("set_point_shape_mode", &guik::ShaderSetting::set_point_shape_mode, py::arg("mode"), py::return_value_policy::reference_internal, "Set point shape mode.")
    .def("set_point_shape_circle", &guik::ShaderSetting::set_point_shape_circle, py::return_value_policy::reference_internal, "Set point shape to circle.")
    .def("set_point_shape_rectangle", &guik::ShaderSetting::set_point_shape_rectangle, py::return_value_policy::reference_internal, "Set point shape to rectangle.")
    .def("set_point_scale_mode", &guik::ShaderSetting::set_point_scale_mode, py::arg("mode"), py::return_value_policy::reference_internal, "Set point size scale mode.")
    .def("set_point_scale_screenspace", &guik::ShaderSetting::set_point_scale_screenspace, py::return_value_policy::reference_internal, "Set point size scale to screen space.")
    .def("set_point_scale_metric", &guik::ShaderSetting::set_point_scale_metric, py::return_value_policy::reference_internal, "Set point size scale to metric.")
    .def(
      "set_point_shape",
      &guik::ShaderSetting::set_point_shape,
      py::arg("point_size"),
      py::arg("metric"),
      py::arg("circle"),
      py::return_value_policy::reference_internal,
      R"doc(Set point shape with common settings.

Args:
    point_size: Point size in meters (if metric) or in pseudo pixels (if screenspace).
    metric: If True, point size is in meters. If False, point size is in pixels.
    circle: If True, point shape is circle. If False, point shape is rectangle.
)doc")
    .def(
      "remove_model_matrix",
      &guik::ShaderSetting::remove_model_matrix,
      py::return_value_policy::reference_internal,
      "Remove the model matrix. Note that this invalidates subsequent model matrix operations (translate, rotate, and scale).")
    .def("color_mode", &guik::ShaderSetting::color_mode, "Get color mode.")
    .def(
      "set_color_mode",
      &guik::ShaderSetting::set_color_mode,
      py::arg("color_mode"),
      py::return_value_policy::reference_internal,
      "Set color mode (0: RAINBOW, 1: FLAT_COLOR, 2: VERTEX_COLOR, 3: TEXTURE_COLOR, 4: VERTEX_COLORMAP).")
    .def("model_matrix", &guik::ShaderSetting::model_matrix, "Get model matrix.")
    .def("translation", &guik::ShaderSetting::translation, "Get translation vector from the model matrix.")
    .def("rotation", &guik::ShaderSetting::rotation, "Get rotation matrix from the model matrix.")
    .def("set_model_matrix", &guik::ShaderSetting::set_model_matrix<Eigen::Matrix4f>, py::arg("model_matrix"), py::return_value_policy::reference_internal, "Set model matrix.")
    .def(
      "transform",
      &guik::ShaderSetting::transform<Eigen::Matrix4f>,
      py::arg("transform"),
      py::return_value_policy::reference_internal,
      "Apply transformation to the model matrix. The transformation is applied after the existing transformation (T_new = T_old * transform).")
    .def(
      "translate",
      [](guik::ShaderSetting& self, float x, float y, float z) { return self.translate(x, y, z); },
      py::arg("x"),
      py::arg("y"),
      py::arg("z"),
      py::return_value_policy::reference_internal,
      "Apply translation to the model matrix.")
    .def(
      "translate",
      [](guik::ShaderSetting& self, const Eigen::Vector3f& translation) { return self.translate(translation); },
      py::arg("translation"),
      py::return_value_policy::reference_internal,
      "Apply translation to the model matrix.")
    .def(
      "rotate",
      [](guik::ShaderSetting& self, float angle, const Eigen::Vector3f& axis) { return self.rotate(angle, axis); },
      py::arg("angle"),
      py::arg("axis"),
      py::return_value_policy::reference_internal,
      "Apply rotation (angle [rad] around axis) to the model matrix.")
    .def(
      "rotate",
      [](guik::ShaderSetting& self, const Eigen::Matrix3f& rotation) { return self.rotate(rotation); },
      py::arg("rotation"),
      py::return_value_policy::reference_internal,
      "Apply rotation (3x3 rotation matrix) to the model matrix.")
    .def(
      "scale",
      [](guik::ShaderSetting& self, float scale) { return self.scale(scale); },
      py::arg("scale"),
      py::return_value_policy::reference_internal,
      "Apply uniform scaling to the model matrix.")
    .def(
      "scale",
      [](guik::ShaderSetting& self, const Eigen::Vector3f& scale) { return self.scale(scale); },
      py::arg("scale"),
      py::return_value_policy::reference_internal,
      "Apply non-uniform scaling to the model matrix.");

  // guik::Rainbow
  py::class_<guik::Rainbow, guik::ShaderSetting, std::shared_ptr<guik::Rainbow>>(
    guik_,
    "Rainbow",
    R"doc(Rainbow coloring scheme that assigns colors to the height (z-value) of pixels of the object.
The coloring band can be changed by setting `z_range` param (e.g., add("z_range", [-3.0, 5.0])).
)doc")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>(), py::arg("model_matrix"))
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::Rainbow(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  // guik::VertexColor
  py::class_<guik::VertexColor, guik::ShaderSetting, std::shared_ptr<guik::VertexColor>>(
    guik_,
    "VertexColor",
    "Vertex color scheme that assigns colors based on vertex color attributes.")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>(), py::arg("model_matrix"))
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::VertexColor(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  // guik::TextureColor
  py::class_<guik::TextureColor, guik::ShaderSetting, std::shared_ptr<guik::TextureColor>>(
    guik_,
    "TextureColor",
    "Texture color scheme that assigns colors based on the bound texture.")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>(), py::arg("model_matrix"))
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::TextureColor(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  // guik::FlatColor
  py::class_<guik::FlatColor, guik::ShaderSetting, std::shared_ptr<guik::FlatColor>>(guik_, "FlatColor", R"doc(Flat coloring scheme that assigns a single color to the object.
If the alpha value is less than 0.999, the object is rendered as transparent.
)doc")
    .def(py::init<float, float, float, float>(), py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"))
    .def(py::init<float, float, float, float, Eigen::Matrix4f>(), py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"), py::arg("model_matrix"))
    .def(
      py::init([](float r, float g, float b, float a, float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::FlatColor(r, g, b, a, mat);
      }),
      py::arg("r"),
      py::arg("g"),
      py::arg("b"),
      py::arg("a"),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  // Flat colors
  define_flat_color<guik::FlatRed>(guik_, "FlatRed");
  define_flat_color<guik::FlatGreen>(guik_, "FlatGreen");
  define_flat_color<guik::FlatBlue>(guik_, "FlatBlue");
  define_flat_color<guik::FlatOrange>(guik_, "FlatOrange");
  define_flat_color<guik::FlatWhite>(guik_, "FlatWhite");
  define_flat_color<guik::FlatGray>(guik_, "FlatGray");
  define_flat_color<guik::FlatDarkGray>(guik_, "FlatDarkGray");
  define_flat_color<guik::FlatBlack>(guik_, "FlatBlack");

  // glk::DrawableContainer::push_back(drawable, shader_setting)
  // Defined here (not in define_glk()) so that guik.ShaderSetting is registered before this binding
  py::reinterpret_borrow<py::class_<glk::DrawableContainer, glk::Drawable, std::shared_ptr<glk::DrawableContainer>>>(py::type::of<glk::DrawableContainer>())
    .def(
      "push_back",
      [](glk::DrawableContainer& container, const py::object& drawable, const guik::ShaderSetting& shader_setting) {
        container.push_back(keep_python_drawable_alive(drawable), shader_setting);
      },
      py::arg("drawable"),
      py::arg("shader_setting"),
      R"doc(Add a drawable to the container with a shader setting.

The shader setting is copied, and later changes to it are not reflected.

Args:
    drawable: Drawable to be added.
    shader_setting: Shader setting to be used when drawing the drawable.
)doc");

  // guik::ModelControl
  py::class_<guik::ModelControl>(guik_, "ModelControl", "UI and gizmo to manipulate a model matrix.")
    .def(py::init<std::string, Eigen::Matrix4f>(), py::arg("label") = "model_control", py::arg("model_matrix") = Eigen::Matrix4f::Identity())
    .def("draw_ui", &guik::ModelControl::draw_ui, "Draw the model control UI window. Call this in a UI callback.")
    .def("draw_gizmo_ui", &guik::ModelControl::draw_gizmo_ui, "Draw the gizmo setting UI. Call this in a UI callback.")
    .def(
      "draw_gizmo",
      [](guik::ModelControl& control) {
        auto viewer = guik::LightViewer::instance();
        control.draw_gizmo(0, 0, viewer->canvas_size().x(), viewer->canvas_size().y(), viewer->view_matrix(), viewer->projection_matrix());
      },
      "Draw the gizmo on the main viewer canvas. Call this in a UI callback.")
    .def("model_matrix", &guik::ModelControl::model_matrix, "Get the model matrix.")
    .def(
      "set_model_matrix",
      [](guik::ModelControl& model_control, const Eigen::Matrix4f& mat) { model_control.set_model_matrix(mat); },
      py::arg("model_matrix"),
      "Set the model matrix.")
    .def("set_gizmo_enabled", &guik::ModelControl::set_gizmo_enabled, py::arg("enabled"))
    .def("enable_gizmo", &guik::ModelControl::enable_gizmo)
    .def("disable_gizmo", &guik::ModelControl::disable_gizmo)
    .def(
      "set_gizmo_operation",
      [](guik::ModelControl& model_control, const std::string& op) { model_control.set_gizmo_operation(op); },
      py::arg("operation"),
      R"doc(Set the gizmo operation.

Args:
    operation: "TRANSLATE", "ROTATE", "SCALE", "SCALEU", or "UNIVERSAL".
)doc")
    .def("set_gizmo_mode", &guik::ModelControl::set_gizmo_mode, py::arg("mode"), "Set the gizmo mode (0: LOCAL, 1: WORLD).")
    .def("set_gizmo_clip_scale", &guik::ModelControl::set_gizmo_clip_scale, py::arg("space") = 0.1f, "Change the gizmo size (default = 0.1).");

  // guik::CameraControl
  py::class_<guik::CameraControl, std::shared_ptr<guik::CameraControl>>(guik_, "CameraControl", R"doc(Base class for camera control.

Many of default camera control methods have an interface that combines a programmatic API (e.g., lookat()) and
user input (e.g., mouse dragging). For example, lookat() can be used to programmatically set the camera to look at
a specific point in the world, while mouse dragging can be used to shift the camera's center of focus.
reset_center() can be used to reset any such shifts.
)doc")
    .def("reset_center", &guik::CameraControl::reset_center, "Reset the shift of the camera.")
    .def(
      "lookat",
      [](guik::CameraControl& control, const Eigen::Vector3f& pt) { control.lookat(pt); },
      py::arg("pt"),
      "Move the camera to look at the given point in world coordinates.")
    .def(
      "lookat",
      [](guik::CameraControl& control, const Eigen::Matrix4f& T_world_sensor) { control.lookat(Eigen::Isometry3f(T_world_sensor)); },
      py::arg("T_world_sensor"),
      R"doc(Move the camera to look at the given transform (4x4 matrix in world coordinates).
The camera looks at the translation part of the transform, and optionally aligns its orientation with the rotation part.
)doc")
    .def("view_matrix", &guik::CameraControl::view_matrix, "Get the 4x4 view matrix of the camera.");
  py::class_<guik::OrbitCameraControlXY, guik::CameraControl, std::shared_ptr<guik::OrbitCameraControlXY>>(guik_, "OrbitCameraControlXY");
  py::class_<guik::OrbitCameraControlXZ, guik::CameraControl, std::shared_ptr<guik::OrbitCameraControlXZ>>(guik_, "OrbitCameraControlXZ");
  py::class_<guik::ArcBallCameraControl, guik::CameraControl, std::shared_ptr<guik::ArcBallCameraControl>>(guik_, "ArcBallCameraControl");
  py::class_<guik::TopDownCameraControl, guik::CameraControl, std::shared_ptr<guik::TopDownCameraControl>>(guik_, "TopDownCameraControl");
  py::class_<guik::FPSCameraControl, guik::CameraControl, std::shared_ptr<guik::FPSCameraControl>>(guik_, "FPSCameraControl", "FPS game-like camera control.")
    .def("set_fovy", &guik::FPSCameraControl::set_fovy, py::arg("fovy_deg"), "Set the vertical field of view [deg].")
    .def("set_depth_range", &guik::FPSCameraControl::set_depth_range, py::arg("range"), "Set the depth range (near, far).")
    .def("set_pose", &guik::FPSCameraControl::set_pose, py::arg("pos"), py::arg("yaw_deg"), py::arg("pitch_deg"), "Set the camera pose.")
    .def(
      "set_mouse_senstivity",
      &guik::FPSCameraControl::set_mouse_senstivity,
      py::arg("mouse_sensitivity_pitch") = 0.01,
      py::arg("mouse_sensitivity_yaw") = 0.01,
      "Set the mouse sensitivity.")
    .def("set_translation_speed", &guik::FPSCameraControl::set_translation_speed, py::arg("speed") = 0.05, "Set the translation speed.");
  py::class_<guik::SensorViewCameraControl, guik::CameraControl, std::shared_ptr<guik::SensorViewCameraControl>>(
    guik_,
    "SensorViewCameraControl",
    R"doc(Camera control that orbits around a tracked sensor pose.
The camera always looks at the sensor position, with user-controlled azimuth/elevation and distance scaling.
)doc")
    .def(
      "set_sensor_pose",
      [](guik::SensorViewCameraControl& control, const Eigen::Isometry3f& T_world_sensor) { control.set_sensor_pose(T_world_sensor); },
      py::arg("T_world_sensor"),
      "Set the raw sensor pose (T_world_sensor, 4x4 matrix).")
    .def(
      "set_sensor_camera_transform",
      [](guik::SensorViewCameraControl& control, const Eigen::Isometry3f& T_sensor_camera) { control.set_sensor_camera_transform(T_sensor_camera); },
      py::arg("T_sensor_camera"),
      "Set the transform from sensor frame to camera frame (T_sensor_camera, 4x4 matrix).")
    .def(
      "set_smoothing_factor",
      &guik::SensorViewCameraControl::set_smoothing_factor,
      py::arg("trans"),
      py::arg("rot"),
      "Set translation and rotation smoothing factors in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).")
    .def(
      "set_z_axis_alignment",
      &guik::SensorViewCameraControl::set_z_axis_alignment,
      py::arg("enabled"),
      "Enable or disable z-axis alignment. When enabled (default), the camera up direction is locked to the world z-axis.");

  py::class_<guik::ProjectionControl, std::shared_ptr<guik::ProjectionControl>>(guik_, "ProjectionControl");
  py::class_<guik::BasicProjectionControl, guik::ProjectionControl, std::shared_ptr<guik::BasicProjectionControl>>(guik_, "BasicProjectionControl");

  // guik::HoveredDrawings
  py::class_<guik::HoveredDrawings, std::shared_ptr<guik::HoveredDrawings>>(guik_, "HoveredDrawings")  //
    .def(
      py::init([](guik::LightViewerContext* context) {
        if (context == nullptr) {
          context = guik::LightViewer::instance();
        }
        return std::make_shared<guik::HoveredDrawings>(context);
      }),
      py::arg("context") = nullptr)
    .def(
      "add_text",
      &guik::HoveredDrawings::add_text,
      py::arg("pt"),
      py::arg("text"),
      py::arg("fg_color") = 0xFFFFFFFF,
      py::arg("bg_color") = 0x80000000,
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def(
      "add_text_on",
      &guik::HoveredDrawings::add_text_on,
      py::arg("drawable_name"),
      py::arg("text"),
      py::arg("fg_color") = 0xFFFFFFFF,
      py::arg("bg_color") = 0x80000000,
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def("add_cross", &guik::HoveredDrawings::add_cross, py::arg("pt"), py::arg("color") = 0xFFFFFFFF, py::arg("size") = 7.07f, py::arg("thickness") = 1.0f)
    .def("add_cross_on", &guik::HoveredDrawings::add_cross_on, py::arg("drawable_name"), py::arg("color") = 0xFFFFFFFF, py::arg("size") = 7.07f, py::arg("thickness") = 1.0f)
    .def(
      "add_circle",
      &guik::HoveredDrawings::add_circle,
      py::arg("pt"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("radius") = 10.0f,
      py::arg("num_segments") = 32,
      py::arg("thickness") = 1.0f)
    .def(
      "add_circle_on",
      &guik::HoveredDrawings::add_circle_on,
      py::arg("drawable_name"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("radius") = 10.0f,
      py::arg("num_segments") = 32,
      py::arg("thickness") = 1.0f)
    .def(
      "add_triangle",
      &guik::HoveredDrawings::add_triangle,
      py::arg("pt"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("height") = 20.0f,
      py::arg("thickness") = 1.0f,
      py::arg("upsidedown") = true,
      py::arg("centering") = false)
    .def(
      "add_triangle_on",
      &guik::HoveredDrawings::add_triangle_on,
      py::arg("drawable"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("height") = 20.0f,
      py::arg("thickness") = 1.0f,
      py::arg("upsidedown") = true,
      py::arg("centering") = false)
    .def(
      "add_filled_triangle",
      &guik::HoveredDrawings::add_filled_triangle,
      py::arg("pt"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("height") = 20.0f,
      py::arg("upsidedown") = true,
      py::arg("centering") = false)
    .def(
      "add_filled_triangle_on",
      &guik::HoveredDrawings::add_filled_triangle_on,
      py::arg("drawable_name"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("height") = 20.0f,
      py::arg("upsidedown") = true,
      py::arg("centering") = false)
    .def(
      "add_rect",
      &guik::HoveredDrawings::add_rect,
      py::arg("pt"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("size") = Eigen::Vector2f(15.0f, 15.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def(
      "add_rect_on",
      &guik::HoveredDrawings::add_rect_on,
      py::arg("drawable"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("size") = Eigen::Vector2f(15.0f, 15.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def(
      "add_filled_rect",
      &guik::HoveredDrawings::add_filled_rect,
      py::arg("pt"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("size") = Eigen::Vector2f(15.0f, 15.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def(
      "add_filled_rect_on",
      &guik::HoveredDrawings::add_filled_rect_on,
      py::arg("drawable_name"),
      py::arg("color") = 0xFFFFFFFF,
      py::arg("size") = Eigen::Vector2f(15.0f, 15.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f))
    .def(
      "add_image",
      &guik::HoveredDrawings::add_image,
      py::arg("pt"),
      py::arg("texture"),
      py::arg("size") = Eigen::Vector2f(0.0f, 0.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f),
      py::arg("bg_color") = 0,
      py::arg("border_color") = 0,
      py::arg("border_thickness") = 1.0f)
    .def(
      "add_image_on",
      &guik::HoveredDrawings::add_image_on,
      py::arg("drawable_name"),
      py::arg("texture"),
      py::arg("size") = Eigen::Vector2f(0.0f, 0.0f),
      py::arg("offset") = Eigen::Vector2f(0.0f, 0.0f),
      py::arg("bg_color") = 0,
      py::arg("border_color") = 0,
      py::arg("border_thickness") = 1.0f)
    .def("remove_drawing", &guik::HoveredDrawings::remove_drawing, py::arg("drawing_id"))
    .def("clear", &guik::HoveredDrawings::clear, "Remove all drawings.")
    .def("create_callback", &guik::HoveredDrawings::create_callback, "Create a UI callback function to draw the drawings. Register it with register_ui_callback().");

  // guik::Application
  py::class_<guik::Application, std::unique_ptr<guik::Application, py::nodelete>>(guik_, "Application")
    .def("ok", &guik::Application::ok)
    .def("enable_vsync", &guik::Application::enable_vsync)
    .def("disable_vsync", &guik::Application::disable_vsync)
    .def("enable_docking", &guik::Application::enable_docking)
    .def("disable_docking", &guik::Application::disable_docking)
    .def("window_size", &guik::Application::window_size)
    .def("show_window", &guik::Application::show_window)
    .def("hide_window", &guik::Application::hide_window)
    .def("maximize_window", &guik::Application::maximize_window)
    .def("fullscreen_window", &guik::Application::fullscreen_window)
    .def("resize", &guik::Application::resize, py::arg("size"))
    .def("set_title", &guik::Application::set_title, py::arg("title"))
    .def("framebuffer_size", &guik::Application::framebuffer_size)
    .def("spin", &guik::Application::spin)
    .def("spin_once", &guik::Application::spin_once)
    .def("close", &guik::Application::close)
    .def("closed", &guik::Application::closed);

  // LightViewerContext
  py::class_<guik::LightViewerContext, std::unique_ptr<guik::LightViewerContext, py::nodelete>>(guik_, "LightViewerContext", "Viewer context (the main viewer or a sub viewer).")
    .def("set_size", &guik::LightViewerContext::set_size, py::arg("size"), "Set the canvas size.")
    .def("set_clear_color", &guik::LightViewerContext::set_clear_color, py::arg("color"), "Set the background clear color (RGBA).")

    .def(
      "set_pos",
      &guik::LightViewerContext::set_pos,
      py::arg("pos"),
      py::arg("cond") = static_cast<int>(ImGuiCond_FirstUseEver),
      py::arg("flags") = 0,
      "Set the position of the sub viewer window.")
    .def("show", &guik::LightViewerContext::show, "Show the sub viewer window.")
    .def("hide", &guik::LightViewerContext::hide, "Hide the sub viewer window.")

    .def("clear", &guik::LightViewerContext::clear, "Clear everything in the viewer (callbacks, filters, drawables, texts).")
    .def("clear_text", &guik::LightViewerContext::clear_text, "Clear texts.")
    .def("append_text", &guik::LightViewerContext::append_text, py::arg("text"), "Append debug text to the viewer. This method is thread-safe.")
    .def(
      "register_ui_callback",
      &guik::LightViewerContext::register_ui_callback,
      py::arg("callback_name"),
      py::arg("callback"),
      R"doc(Register a UI callback function that is called every frame to draw ImGui UI.

Args:
    callback_name: Callback name. If a callback with the same name exists, it is replaced.
    callback: Callback function. If None, the callback with the given name is removed.
)doc")
    .def("remove_ui_callback", &guik::LightViewerContext::remove_ui_callback, py::arg("callback_name"), "Remove a UI callback function.")

    .def(
      "shader_setting",
      [](guik::LightViewerContext& context) { return &context.shader_setting(); },
      py::return_value_policy::reference,
      "Global shader setting applied to all drawables.")

    .def("disable_xy_grid", &guik::LightViewerContext::disable_xy_grid, "Disable XY grid drawing.")
    .def("enable_xy_grid", &guik::LightViewerContext::enable_xy_grid, "Enable XY grid drawing.")
    .def("set_draw_xy_grid", &guik::LightViewerContext::set_draw_xy_grid, py::arg("draw_xy_grid"), "Set enable/disable XY grid drawing.")
    .def("set_colormap", &guik::LightViewerContext::set_colormap, py::arg("colormap"), "Set colormap used for RAINBOW and VERTEX_COLORMAP modes.")
    .def("set_screen_effect", &guik::LightViewerContext::set_screen_effect, py::arg("effect"), "Set screen effect.")
    .def("get_screen_effect", &guik::LightViewerContext::get_screen_effect, "Get screen effect.")
    .def("set_bg_texture", &guik::LightViewerContext::set_bg_texture, py::arg("bg_texture"), "Set background texture.")

    .def("enable_normal_buffer", &guik::LightViewerContext::enable_normal_buffer, "Enable normal buffer.")
    .def("enable_info_buffer", &guik::LightViewerContext::enable_info_buffer, "Enable info buffer (required for pick_info()).")
    .def(
      "enable_partial_rendering",
      &guik::LightViewerContext::enable_partial_rendering,
      py::arg("clear_thresh") = 1e-6,
      R"doc(Enable partial rendering.

Args:
    clear_thresh: Threshold for clearing the partial rendering buffer.
)doc")
    .def("disable_partial_rendering", &guik::LightViewerContext::disable_partial_rendering, "Disable partial rendering.")
    .def("enable_backface_culling", &guik::LightViewerContext::enable_backface_culling, "Enable backface culling. Fragments with back-facing normals will not be rendered.")
    .def("disable_backface_culling", &guik::LightViewerContext::disable_backface_culling, "Disable backface culling.")
    .def(
      "set_backface_culling_range",
      [](guik::LightViewerContext& context, float min_depth, float max_depth) { context.set_backface_culling_range(Eigen::Vector2f(min_depth, max_depth)); },
      py::arg("min_depth"),
      py::arg("max_depth"),
      "Set the [min, max] range of normal.z for backface culling. Fragments with normal.z outside the range are culled when backface culling is enabled.")
    .def("backface_culling_enabled", &guik::LightViewerContext::backface_culling_enabled, "Whether backface culling is enabled.")

    .def("color_buffer", &guik::LightViewerContext::color_buffer, py::return_value_policy::reference_internal, "Color buffer texture.")
    .def("depth_buffer", &guik::LightViewerContext::depth_buffer, py::return_value_policy::reference_internal, "Depth buffer texture.")
    .def(
      "normal_buffer",
      [](const guik::LightViewerContext& context) -> const glk::Texture& {
        if (!context.normal_buffer_enabled()) {
          throw std::runtime_error("normal buffer is not enabled. Call enable_normal_buffer() first");
        }
        return context.normal_buffer();
      },
      py::return_value_policy::reference_internal,
      "Normal buffer texture. enable_normal_buffer() must be called beforehand.")
    .def(
      "info_buffer",
      [](const guik::LightViewerContext& context) -> const glk::Texture& {
        if (!context.info_buffer_enabled()) {
          throw std::runtime_error("info buffer is not enabled. Call enable_info_buffer() first");
        }
        return context.info_buffer();
      },
      py::return_value_policy::reference_internal,
      "Info buffer texture. enable_info_buffer() must be called beforehand.")

    .def(
      "clear_drawables",
      [](guik::LightViewerContext& context) { context.clear_drawables(); },
      "Clear all drawables.")

    .def("get_drawables", &guik::LightViewerContext::get_drawables, "Get all drawables as a dict of name -> (shader_setting, drawable). This method may be removed in the future.")
    .def(
      "find_drawable",
      &guik::LightViewerContext::find_drawable,
      py::arg("name"),
      "Find a drawable by name. Returns a (shader_setting, drawable) pair if found, otherwise (None, None).")
    .def(
      "remove_drawable",
      [](guik::LightViewerContext& context, const std::string& pattern, bool regex) {
        if (regex) {
          context.remove_drawable(std::regex(pattern));
        } else {
          context.remove_drawable(pattern);
        }
      },
      py::arg("pattern"),
      py::arg("regex") = false,
      R"doc(Remove drawables.

Args:
    pattern: Drawable name, or a regular expression if regex is True.
    regex: If True, all drawables whose names match pattern are removed.
)doc")
    .def(
      "update_drawable",
      [](guik::LightViewerContext& context, const std::string& name, const py::object& drawable, const guik::ShaderSetting& shader_setting) {
        context.update_drawable(name, keep_python_drawable_alive(drawable), shader_setting);
      },
      py::arg("name"),
      py::arg("drawable"),
      py::arg("shader_setting") = guik::ShaderSetting(),
      R"doc(Register or update a drawable.

Args:
    name: Drawable name. If a drawable with the same name exists, it is replaced.
    drawable: Drawable.
    shader_setting: Shader setting. It is copied, and later changes to it are not reflected.
)doc")

    .def("clear_drawable_filters", &guik::LightViewerContext::clear_drawable_filters, "Clear all drawable filters.")
    .def(
      "register_drawable_filter",
      &guik::LightViewerContext::register_drawable_filter,
      py::arg("filter_name"),
      py::arg("filter"),
      R"doc(Register a drawable filter. Drawables for which filter(name) returns False are not rendered.

Args:
    filter_name: Filter name.
    filter: Filter function that takes a drawable name and returns a bool. If None, the filter with the given name is removed.
)doc")
    .def("remove_drawable_filter", &guik::LightViewerContext::remove_drawable_filter, py::arg("filter_name"), "Remove a drawable filter.")

    .def("clear_partial_rendering", &guik::LightViewerContext::clear_partial_rendering, "Clear partial rendering buffer.")

    .def("get_camera_control", &guik::LightViewerContext::get_camera_control, "Get camera control (view matrix control).")
    .def("set_camera_control", &guik::LightViewerContext::set_camera_control, py::arg("camera_control"), "Set camera control (view matrix control).")
    .def("get_projection_control", &guik::LightViewerContext::get_projection_control, "Get projection control (projection matrix control).")
    .def("set_projection_control", &guik::LightViewerContext::set_projection_control, py::arg("projection_control"), "Set projection control (projection matrix control).")

    .def("save_camera_settings", &guik::LightViewerContext::save_camera_settings, py::arg("path"), "Save camera settings to a file.")
    .def("load_camera_settings", &guik::LightViewerContext::load_camera_settings, py::arg("path"), "Load camera settings from a file.")

    .def("save_color_buffer", &guik::LightViewerContext::save_color_buffer, py::arg("path"), "Save color buffer to an image file (PNG).")
    .def(
      "save_depth_buffer",
      &guik::LightViewerContext::save_depth_buffer,
      py::arg("path"),
      py::arg("real_scale") = true,
      "Save depth buffer to a file (PNG). If real_scale is True, depth values are scaled to real-world values.")

    .def("reset_center", &guik::LightViewerContext::reset_center, "Reset camera center to the center of the screen.")
    .def(
      "lookat",
      [](guik::LightViewerContext& context, const Eigen::Vector3f& pt) { context.lookat(pt); },
      py::arg("pt"),
      "Make the camera look at a point.")
    .def(
      "use_orbit_camera_control",
      &guik::LightViewerContext::use_orbit_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0,
      "Use orbit camera control (XY plane) useful for systems in which XYZ = front-left-up like LiDAR odometry.")
    .def(
      "use_orbit_camera_control_xz",
      &guik::LightViewerContext::use_orbit_camera_control_xz,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = 0.0,
      "Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.")
    .def(
      "use_topdown_camera_control",
      &guik::LightViewerContext::use_topdown_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      "Use top-down camera control useful for 2D SLAM.")
    .def(
      "use_arcball_camera_control",
      &guik::LightViewerContext::use_arcball_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0,
      "Use arcball camera control.")
    .def("use_fps_camera_control", &guik::LightViewerContext::use_fps_camera_control, py::arg("fovy_deg") = 60.0, "Use FPS game-like camera control.")
    .def(
      "use_sensor_view_camera_control",
      &guik::LightViewerContext::use_sensor_view_camera_control,
      py::arg("T_sensor_camera"),
      py::arg("smoothing_factor_trans") = 0.75,
      py::arg("smoothing_factor_rot") = 0.9,
      R"doc(Use sensor-view camera control that follows a given sensor pose (set via SensorViewCameraControl.set_sensor_pose()).

Args:
    T_sensor_camera: Initial transform from sensor frame to camera frame (4x4 matrix).
    smoothing_factor_trans: Smoothing factor for translation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
    smoothing_factor_rot: Smoothing factor for rotation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
)doc")

    .def(
      "use_perspective_projection_control",
      &guik::LightViewerContext::use_perspective_projection_control,
      py::arg("fovy_deg") = 30.0f,
      py::arg("near") = 1.0f,
      py::arg("far") = 1000.0f,
      "Use perspective projection control.")
    .def(
      "use_orthographic_projection_control",
      &guik::LightViewerContext::use_orthographic_projection_control,
      py::arg("width") = 10.0f,
      py::arg("near") = 1.0f,
      py::arg("far") = 1000.0f,
      "Use orthographic projection control.")

    .def(
      "set_point_shape",
      &guik::LightViewerContext::set_point_shape,
      py::arg("point_size") = 1.0f,
      py::arg("metric") = true,
      py::arg("circle") = true,
      R"doc(Set point shape properties.

Args:
    point_size: Point size.
    metric: If True, point size is in the metric unit [m], otherwise in pixel unit.
    circle: If True, points are rendered as circles, otherwise as squares.
)doc")

    .def(
      "pick_info",
      &guik::LightViewerContext::pick_info,
      py::arg("p"),
      py::arg("window") = 2,
      R"doc(Pick info from the info buffer. enable_info_buffer() must be called beforehand.

Args:
    p: Pixel coordinates.
    window: Window size for finding a valid pixel neighboring p.

Returns:
    Info vector (4 ints).
)doc")
    .def(
      "pick_depth",
      &guik::LightViewerContext::pick_depth,
      py::arg("p"),
      py::arg("window") = 2,
      R"doc(Pick depth from the depth buffer.

Args:
    p: Pixel coordinates.
    window: Window size for finding a valid pixel neighboring p.

Returns:
    Depth value. Depth > 1.0 indicates background.
)doc")
    .def(
      "pick_point",
      [](guik::LightViewerContext& context, int button, int window) {
        const auto& io = ImGui::GetIO();
        const int mouse_button_count = static_cast<int>(IM_ARRAYSIZE(io.MouseClicked));
        if (button < 0 || button >= mouse_button_count) {
          throw py::value_error("button out of range");
        }
        return context.pick_point(button, window);
      },
      py::arg("button") = 0,
      py::arg("window") = 2,
      R"doc(Pick a 3D point under the mouse cursor when the mouse button is clicked. Call this in a UI callback.

Args:
    button: Mouse button index (0: left, 1: right, 2: middle).
    window: Window size for finding a valid pixel neighboring the mouse position.

Returns:
    3D point in the world coordinate, or None if the button is not clicked or no valid depth is found.
)doc")
    .def(
      "unproject",
      &guik::LightViewerContext::unproject,
      py::arg("p"),
      py::arg("depth"),
      R"doc(Unproject a pixel to a 3D point.

Args:
    p: Pixel coordinates.
    depth: Depth value (e.g., obtained by pick_depth()).

Returns:
    3D point in the world coordinate.
)doc")

    .def(
      "update_points",
      [](guik::LightViewerContext& context, const std::string& name, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points, const guik::ShaderSetting& shader_setting)
        -> std::shared_ptr<glk::PointCloudBuffer> {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return nullptr;
        }
        return context.update_points(name, points.data(), sizeof(float) * points.cols(), points.rows(), shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("shader_setting"),
      R"doc(Register or update a point cloud buffer.

Args:
    name: Drawable name.
    points: Point positions (N x 3 or N x 4 array).
    shader_setting: Shader setting.

Returns:
    Created PointCloudBuffer, or None if points has an invalid shape.
)doc")

    .def(
      "update_normal_dists",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Matrix3f>& covs,
        float sigma,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), sigma, shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("covs"),
      py::arg("sigma"),
      py::arg("shader_setting"),
      R"doc(Register or update a normal distribution set (rendered as ellipsoids).

Args:
    name: Drawable name.
    points: Point means.
    covs: Point covariances. Must have the same size as points.
    sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
    shader_setting: Shader setting.
)doc")

    .def(
      "update_normal_dists",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector4f>& points,
        const std::vector<Eigen::Matrix4f>& covs,
        float sigma,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), sigma, shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("covs"),
      py::arg("sigma"),
      py::arg("shader_setting"),
      R"doc(Register or update a normal distribution set (rendered as ellipsoids).

Args:
    name: Drawable name.
    points: Point means.
    covs: Point covariances. Must have the same size as points.
    sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
    shader_setting: Shader setting.
)doc")

    .def(
      "update_thin_lines",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& vertices,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& colors,
        const std::vector<unsigned int>& indices,
        bool line_strip,
        const guik::ShaderSetting& shader_setting) -> std::shared_ptr<glk::ThinLines> {
        if (vertices.cols() != 3 && vertices.cols() != 4) {
          std::cerr << "warning: vertices must be Nx3 or Nx4" << std::endl;
          return nullptr;
        }
        if (colors.size()) {
          if (colors.rows() != vertices.rows()) {
            std::cerr << "warning: colors must have the same number of rows as vertices" << std::endl;
            return nullptr;
          }
          if (colors.cols() != 4) {
            std::cerr << "warning: colors must be Nx4" << std::endl;
            return nullptr;
          }
        }

        if (vertices.cols() == 3) {
          return context.update_thin_lines(name, vertices.data(), colors.data(), vertices.rows(), indices.data(), indices.size(), line_strip, shader_setting);
        } else {
          return context.update_thin_lines(
            name,
            reinterpret_cast<const Eigen::Vector4f*>(vertices.data()),
            reinterpret_cast<const Eigen::Vector4f*>(colors.data()),
            vertices.rows(),
            indices.data(),
            indices.size(),
            line_strip,
            shader_setting);
        }
      },
      py::arg("name"),
      py::arg("vertices"),
      py::arg("colors") = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>(),
      py::arg("indices") = std::vector<unsigned int>(),
      py::arg("line_strip") = false,
      py::arg("shader_setting") = guik::ShaderSetting(),
      R"doc(Register or update a thin lines drawable.

Args:
    name: Drawable name.
    vertices: Vertex positions (N x 3 or N x 4 array).
    colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
    indices: Vertex indices. If empty, non-indexed drawing is used.
    line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
    shader_setting: Shader setting.
)doc")

    .def(
      "update_thin_lines",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& vertices,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& colors,
        const std::vector<unsigned int>& indices,
        bool line_strip,
        float line_width,
        const guik::ShaderSetting& shader_setting) -> std::shared_ptr<glk::ThinLines> {
        if (vertices.cols() != 3 && vertices.cols() != 4) {
          std::cerr << "warning: vertices must be Nx3 or Nx4" << std::endl;
          return nullptr;
        }
        if (colors.size()) {
          if (colors.rows() != vertices.rows()) {
            std::cerr << "warning: colors must have the same number of rows as vertices" << std::endl;
            return nullptr;
          }
          if (colors.cols() != 4) {
            std::cerr << "warning: colors must be Nx4" << std::endl;
            return nullptr;
          }
        }

        const float* color_ptr = colors.size() ? colors.data() : nullptr;
        const unsigned int* index_ptr = indices.empty() ? nullptr : indices.data();

        if (vertices.cols() == 3) {
          return context.update_thin_lines(name, vertices.data(), color_ptr, vertices.rows(), index_ptr, indices.size(), line_strip, line_width, shader_setting);
        } else {
          return context.update_thin_lines(
            name,
            reinterpret_cast<const Eigen::Vector4f*>(vertices.data()),
            reinterpret_cast<const Eigen::Vector4f*>(color_ptr),
            vertices.rows(),
            index_ptr,
            indices.size(),
            line_strip,
            line_width,
            shader_setting);
        }
      },
      py::arg("name"),
      py::arg("vertices"),
      py::arg("colors") = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>(),
      py::arg("indices") = std::vector<unsigned int>(),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f,
      py::arg("shader_setting") = guik::ShaderSetting(),
      R"doc(Register or update a thin lines drawable.

Args:
    name: Drawable name.
    vertices: Vertex positions (N x 3 or N x 4 array).
    colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
    indices: Vertex indices. If empty, non-indexed drawing is used.
    line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
    line_width: Width of the lines in pixels.
    shader_setting: Shader setting.
)doc")

    .def("update_icosahedron", &guik::LightViewerContext::update_icosahedron, py::arg("name"), py::arg("shader_setting"), "Register or update an icosahedron primitive.")
    .def("update_sphere", &guik::LightViewerContext::update_sphere, py::arg("name"), py::arg("shader_setting"), "Register or update a sphere primitive.")
    .def("update_cube", &guik::LightViewerContext::update_cube, py::arg("name"), py::arg("shader_setting"), "Register or update a cube primitive.")
    .def("update_cone", &guik::LightViewerContext::update_cone, py::arg("name"), py::arg("shader_setting"), "Register or update a cone primitive.")
    .def("update_frustum", &guik::LightViewerContext::update_frustum, py::arg("name"), py::arg("shader_setting"), "Register or update a frustum primitive.")
    .def("update_coord", &guik::LightViewerContext::update_coord, py::arg("name"), py::arg("shader_setting"), "Register or update a coordinate axes primitive.")

    .def(
      "update_wire_icosahedron",
      &guik::LightViewerContext::update_wire_icosahedron,
      py::arg("name"),
      py::arg("shader_setting"),
      "Register or update a wireframe icosahedron primitive.")
    .def("update_wire_sphere", &guik::LightViewerContext::update_wire_sphere, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe sphere primitive.")
    .def("update_wire_cube", &guik::LightViewerContext::update_wire_cube, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe cube primitive.")
    .def("update_wire_cone", &guik::LightViewerContext::update_wire_cone, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe cone primitive.")
    .def("update_wire_frustum", &guik::LightViewerContext::update_wire_frustum, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe frustum primitive.")

    ;

  // LightViewer
  py::class_<guik::LightViewer, guik::Application, guik::LightViewerContext, std::unique_ptr<guik::LightViewer, py::nodelete>>(
    guik_,
    "LightViewer",
    "Light viewer. The main visualization class in Iridescence. Use guik.viewer() or LightViewer.instance() to get the singleton instance.")
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false,
      py::arg("title") = "screen",
      R"doc(Get the singleton instance of LightViewer. The viewer instance is created upon the first call.

Args:
    size: Initial canvas size. Ignored if the viewer has already been created.
    background: If True, the viewer window is created in the background (invisible).
    title: Window title.
)doc")
    .def(
      "sub_viewer",
      [](guik::LightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      },
      py::arg("name"),
      py::arg("size") = std::make_tuple(-1, -1),
      R"doc(Create or get a sub viewer context.

Args:
    name: Name of the sub viewer context.
    size: Canvas size of the sub viewer. If (-1, -1), the default size is used.
)doc")

    .def("spin_until_click", &guik::LightViewer::spin_until_click, "Spin the viewer until a mouse click event is received.")
    .def("toggle_spin_once", &guik::LightViewer::toggle_spin_once, "Spin the viewer until a toggle flag is set to true.")
    .def("invoke", &guik::LightViewer::invoke, py::arg("func"), "Invoke a task in the visualization thread.")
    .def("invoke_after_rendering", &guik::LightViewer::invoke_after_rendering, py::arg("func"), "Invoke a task after rendering in the visualization thread.")
    .def("invoke_once", &guik::LightViewer::invoke_once, py::arg("label"), py::arg("func"), "Invoke a labeled task only once in the life time of the viewer.")

    .def("clear_images", &guik::LightViewer::clear_images, "Clear all images.")
    .def("remove_image", &guik::LightViewer::remove_image, py::arg("name"), "Remove an image by name.")
    .def(
      "update_image",
      &guik::LightViewer::update_image,
      py::arg("name"),
      py::arg("image"),
      py::arg("scale") = -1.0,
      py::arg("order") = -1,
      R"doc(Register or update an image.

Args:
    name: Name of the image.
    image: Image texture.
    scale: Scale factor for displaying the image. If negative, auto scale is used.
    order: Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
)doc")
    .def("clear_plots", &guik::LightViewer::clear_plots, py::arg("clear_settings") = true, "Clear all plots. If clear_settings is True, plot settings are cleared as well.")
    .def(
      "remove_plot",
      &guik::LightViewer::remove_plot,
      py::arg("plot_name"),
      py::arg("label") = "",
      R"doc(Remove a plot by name.

Args:
    plot_name: Name of the plot to be removed.
    label: If non-empty, remove only the plot with the specified label in the plot group.
)doc")
    .def(
      "setup_plot",
      &guik::LightViewer::setup_plot,
      py::arg("plot_name"),
      py::arg("width"),
      py::arg("height"),
      py::arg("plot_flags") = 0,
      py::arg("x_flags") = 0,
      py::arg("y_flags") = 0,
      py::arg("order") = -1,
      R"doc(Setup a plot.

Args:
    plot_name: Name of the plot.
    width: Width of the plot [pixels].
    height: Height of the plot [pixels].
    plot_flags: Flags for the plot (ImPlotFlags).
    x_flags: Flags for the x-axis (ImPlotAxisFlags).
    y_flags: Flags for the y-axis (ImPlotAxisFlags).
    order: Display order of the plot. Plots with smaller order values are displayed first. If -1, the plot is added to the end of the display order.
)doc")
    .def(
      "link_plot_axis",
      &guik::LightViewer::link_plot_axis,
      py::arg("plot_name"),
      py::arg("link_id"),
      py::arg("axis"),
      R"doc(Link an axis of a plot to a link ID. Axes with the same link ID share their ranges.

Args:
    plot_name: Name of the plot.
    link_id: Link ID.
    axis: Axis to be linked (ImAxis: X1 = 0, X2 = 1, X3 = 2, Y1 = 3, Y2 = 4, Y3 = 5).
)doc")
    .def(
      "link_plot_axes",
      &guik::LightViewer::link_plot_axes,
      py::arg("plot_name"),
      py::arg("link_id"),
      py::arg("axes") = -1,
      R"doc(Link multiple axes of a plot to a link ID. Axes with the same link ID share their ranges.

Args:
    plot_name: Name of the plot.
    link_id: Link ID.
    axes: Bitwise OR of axes to be linked ((1 << ImAxis_X1) | (1 << ImAxis_Y1) ...). If -1, all axes are linked.
)doc")
    .def(
      "setup_legend",
      &guik::LightViewer::setup_legend,
      py::arg("plot_name"),
      py::arg("loc"),
      py::arg("flags") = 0,
      R"doc(Setup the legend for a plot.

Args:
    plot_name: Name of the plot.
    loc: Location of the legend (ImPlotLocation).
    flags: Flags for the legend (ImPlotLegendFlags).
)doc")
    .def(
      "setup_plot_group_order",
      &guik::LightViewer::setup_plot_group_order,
      py::arg("group_name"),
      py::arg("order"),
      R"doc(Setup the plot group order.

Args:
    group_name: Name of the plot group.
    order: Display order of the plot group. Plot groups with smaller order values are displayed first.
)doc")

    .def(
      "update_plot_line",
      [](
        guik::LightViewer& viewer,
        const std::string& plot_name,
        const std::string& label,
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        int line_flags,
        int max_num_data) {
        if (ys.empty()) {
          viewer.update_plot_line(plot_name, label, xs, line_flags, max_num_data);
        } else {
          viewer.update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("line_flags") = 0,
      py::arg("max_num_data") = 8192 * 128,
      R"doc(Update a line plot.

Args:
    plot_name: Name of the plot.
    label: Label.
    xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
    ys: Y data.
    line_flags: Flags for the line plot (ImPlotLineFlags).
    max_num_data: Maximum number of data points to be displayed.
)doc")

    .def(
      "update_plot_scatter",
      [](guik::LightViewer& viewer, const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags) {
        if (ys.empty()) {
          viewer.update_plot_scatter(plot_name, label, xs, scatter_flags);
        } else {
          viewer.update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("scatter_flags") = 0,
      R"doc(Update a scatter plot.

Args:
    plot_name: Name of the plot.
    label: Label.
    xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
    ys: Y data.
    scatter_flags: Flags for the scatter plot (ImPlotScatterFlags).
)doc")

    .def(
      "update_plot_histogram",
      [](
        guik::LightViewer& viewer,
        const std::string& plot_name,
        const std::string& label,
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        int x_bins,
        int y_bins,
        const Eigen::Vector2d& x_range,
        const Eigen::Vector2d& y_range,
        int histogram_flags) {
        if (ys.empty()) {
          viewer.update_plot_histogram(plot_name, label, xs, x_bins, x_range, histogram_flags);
        } else {
          viewer.update_plot_histogram(plot_name, label, xs, ys, x_bins, y_bins, x_range, y_range, histogram_flags);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("x_bins") = -2,
      py::arg("y_bins") = -2,
      py::arg("x_range") = Eigen::Vector2d(0.0, 0.0),
      py::arg("y_range") = Eigen::Vector2d(0.0, 0.0),
      py::arg("histogram_flags") = 0,
      R"doc(Update a histogram plot. If ys is empty, a 1D histogram of xs is plotted. Otherwise, a 2D histogram of (xs, ys) is plotted.

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
)doc")
    .def(
      "update_plot_stairs",
      [](guik::LightViewer& viewer, const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int stairs_flags) {
        if (ys.empty()) {
          viewer.update_plot_stairs(plot_name, label, xs, stairs_flags);
        } else {
          if (xs.size() != ys.size()) {
            throw py::value_error("update_plot_stairs requires xs and ys to have the same length");
          }
          viewer.update_plot_stairs(plot_name, label, xs, ys, stairs_flags);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("stairs_flags") = 0,
      R"doc(Update a stairs plot.

Args:
    plot_name: Name of the plot.
    label: Label.
    xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
    ys: Y data. Must have the same length as xs.
    stairs_flags: Flags for the stairs plot (ImPlotStairsFlags).
)doc")

    .def("fit_plot", &guik::LightViewer::fit_plot, py::arg("plot_name"), "Fit the plot view to the data range.")
    .def("fit_all_plots", &guik::LightViewer::fit_all_plots, "Fit all plots to their data ranges.")

    ;

  // AsyncLightViewerContext
  py::class_<guik::AsyncLightViewerContext, std::unique_ptr<guik::AsyncLightViewerContext, py::nodelete>>(
    guik_,
    "AsyncLightViewerContext",
    R"doc(Async interface to manipulate LightViewerContext.
All OpenGL operations are called in the viewer thread through invoke().
Note that async operations come at a cost of data copy.
)doc")
    .def("clear", &guik::AsyncLightViewerContext::clear, "Clear everything in the viewer (callbacks, filters, drawables, texts).")
    .def("clear_text", &guik::AsyncLightViewerContext::clear_text, "Clear texts.")
    .def("append_text", &guik::AsyncLightViewerContext::append_text, py::arg("text"), "Append debug text to the viewer. This method is thread-safe.")
    .def(
      "register_ui_callback",
      &guik::AsyncLightViewerContext::register_ui_callback,
      py::arg("callback_name"),
      py::arg("callback"),
      R"doc(Register a UI callback function that is called every frame in the viewer thread to draw ImGui UI.

Args:
    callback_name: Callback name. If a callback with the same name exists, it is replaced.
    callback: Callback function. If None, the callback with the given name is removed.
)doc")
    .def("remove_ui_callback", &guik::AsyncLightViewerContext::remove_ui_callback, py::arg("callback_name"), "Remove a UI callback function.")

    .def("disable_xy_grid", &guik::AsyncLightViewerContext::disable_xy_grid, "Disable XY grid drawing.")
    .def("enable_xy_grid", &guik::AsyncLightViewerContext::enable_xy_grid, "Enable XY grid drawing.")
    .def("set_draw_xy_grid", &guik::AsyncLightViewerContext::set_draw_xy_grid, py::arg("draw_xy_grid"), "Set enable/disable XY grid drawing.")
    .def("set_colormap", &guik::AsyncLightViewerContext::set_colormap, py::arg("colormap"), "Set colormap used for RAINBOW and VERTEX_COLORMAP modes.")

    .def(
      "set_point_shape",
      &guik::AsyncLightViewerContext::set_point_shape,
      py::arg("point_size") = 1.0f,
      py::arg("metric") = true,
      py::arg("circle") = true,
      R"doc(Set point shape properties.

Args:
    point_size: Point size.
    metric: If True, point size is in the metric unit [m], otherwise in pixel unit.
    circle: If True, points are rendered as circles, otherwise as squares.
)doc")

    .def(
      "clear_drawables",
      [](guik::AsyncLightViewerContext& context) { context.clear_drawables(); },
      "Clear all drawables.")
    .def(
      "remove_drawable",
      [](guik::AsyncLightViewerContext& context, const std::string& pattern, bool regex) {
        if (regex) {
          context.remove_drawable(std::regex(pattern));
        } else {
          context.remove_drawable(pattern);
        }
      },
      py::arg("pattern"),
      py::arg("regex") = false,
      R"doc(Remove drawables.

Args:
    pattern: Drawable name, or a regular expression if regex is True.
    regex: If True, all drawables whose names match pattern are removed.
)doc")

    .def("save_camera_settings", &guik::AsyncLightViewerContext::save_camera_settings, py::arg("path"), "Save camera settings to a file.")
    .def("load_camera_settings", &guik::AsyncLightViewerContext::load_camera_settings, py::arg("path"), "Load camera settings from a file.")
    .def("save_color_buffer", &guik::AsyncLightViewerContext::save_color_buffer, py::arg("path"), "Save color buffer to an image file (PNG).")
    .def(
      "save_depth_buffer",
      &guik::AsyncLightViewerContext::save_depth_buffer,
      py::arg("path"),
      py::arg("real_scale") = true,
      "Save depth buffer to a file (PNG). If real_scale is True, depth values are scaled to real-world values.")

    .def("reset_center", &guik::AsyncLightViewerContext::reset_center, "Reset camera center to the center of the screen.")
    .def(
      "lookat",
      [](guik::AsyncLightViewerContext& context, const Eigen::Vector3f& pt) { context.lookat(pt); },
      py::arg("pt"),
      "Make the camera look at a point.")

    .def(
      "use_orbit_camera_control",
      &guik::AsyncLightViewerContext::use_orbit_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0,
      "Use orbit camera control (XY plane) useful for systems in which XYZ = front-left-up like LiDAR odometry.")
    .def(
      "use_orbit_camera_control_xz",
      &guik::AsyncLightViewerContext::use_orbit_camera_control_xz,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = 0.0,
      "Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.")
    .def(
      "use_topdown_camera_control",
      &guik::AsyncLightViewerContext::use_topdown_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      "Use top-down camera control useful for 2D SLAM.")
    .def(
      "use_arcball_camera_control",
      &guik::AsyncLightViewerContext::use_arcball_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0,
      "Use arcball camera control.")
    .def("use_fps_camera_control", &guik::AsyncLightViewerContext::use_fps_camera_control, py::arg("fovy_deg") = 60.0, "Use FPS game-like camera control.")
    .def(
      "use_sensor_view_camera_control",
      &guik::AsyncLightViewerContext::use_sensor_view_camera_control,
      py::arg("T_sensor_camera"),
      py::arg("smoothing_factor_trans") = 0.75,
      py::arg("smoothing_factor_rot") = 0.9,
      R"doc(Use TPS-like sensor-view camera control.

Args:
    T_sensor_camera: Initial transform from sensor frame to camera frame (4x4 matrix).
    smoothing_factor_trans: Smoothing factor for translation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
    smoothing_factor_rot: Smoothing factor for rotation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
)doc")

    .def(
      "use_perspective_projection_control",
      &guik::AsyncLightViewerContext::use_perspective_projection_control,
      py::arg("fovy_deg") = 30.0f,
      py::arg("near") = 1.0f,
      py::arg("far") = 1000.0f,
      "Use perspective projection control.")
    .def(
      "use_orthographic_projection_control",
      &guik::AsyncLightViewerContext::use_orthographic_projection_control,
      py::arg("width") = 10.0f,
      py::arg("near") = 1.0f,
      py::arg("far") = 1000.0f,
      "Use orthographic projection control.")

    .def("update_drawable_setting", &guik::AsyncLightViewerContext::update_drawable_setting, py::arg("name"), py::arg("setting"), "Update a drawable's shader setting.")

    .def(
      "update_points",
      [](guik::AsyncLightViewerContext& context, const std::string& name, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points, const guik::ShaderSetting& shader_setting) {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return;
        }
        context.update_points(name, points.data(), sizeof(float) * points.cols(), points.rows(), shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("shader_setting"),
      R"doc(Register or update a point cloud buffer.

Args:
    name: Drawable name.
    points: Point positions (N x 3 or N x 4 array).
    shader_setting: Shader setting.
)doc")

    .def(
      "update_points",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& colors,
        const guik::ShaderSetting& shader_setting) {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return;
        }
        if (colors.cols() != 3 && colors.cols() != 4) {
          std::cerr << "warning: colors must be Nx3 or Nx4" << std::endl;
          return;
        }

        context.update_points(name, points.data(), sizeof(float) * points.cols(), colors.data(), sizeof(float) * colors.cols(), points.rows(), shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("colors"),
      py::arg("shader_setting"),
      R"doc(Register or update a point cloud buffer with colors.

Args:
    name: Drawable name.
    points: Point positions (N x 3 or N x 4 array).
    colors: Point colors (N x 3 or N x 4 array).
    shader_setting: Shader setting.
)doc")
    .def(
      "update_normal_dists",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Matrix3f>& covs,
        float sigma,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), sigma, shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("covs"),
      py::arg("sigma"),
      py::arg("shader_setting"),
      R"doc(Register or update a normal distribution set (rendered as ellipsoids).

Args:
    name: Drawable name.
    points: Point means.
    covs: Point covariances. Must have the same size as points.
    sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
    shader_setting: Shader setting.
)doc")

    .def(
      "update_normal_dists",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector4f>& points,
        const std::vector<Eigen::Matrix4f>& covs,
        float sigma,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), sigma, shader_setting);
      },
      py::arg("name"),
      py::arg("points"),
      py::arg("covs"),
      py::arg("sigma"),
      py::arg("shader_setting"),
      R"doc(Register or update a normal distribution set (rendered as ellipsoids).

Args:
    name: Drawable name.
    points: Point means.
    covs: Point covariances. Must have the same size as points.
    sigma: Size of the ellipsoids in standard deviations (e.g., 3.0 renders 3-sigma ellipsoids).
    shader_setting: Shader setting.
)doc")

    .def(
      "update_thin_lines",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& vertices,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& colors,
        const std::vector<unsigned int>& indices,
        bool line_strip,
        const guik::ShaderSetting& shader_setting) {
        if (vertices.cols() != 3 && vertices.cols() != 4) {
          std::cerr << "warning: vertices must be Nx3 or Nx4" << std::endl;
          return;
        }
        if (colors.size()) {
          if (colors.rows() != vertices.rows()) {
            std::cerr << "warning: colors must have the same number of rows as vertices" << std::endl;
            return;
          }
          if (colors.cols() != 4) {
            std::cerr << "warning: colors must be Nx4" << std::endl;
            return;
          }
        }

        if (vertices.cols() == 3) {
          context.update_thin_lines(name, vertices.data(), colors.data(), vertices.rows(), indices.data(), indices.size(), line_strip, shader_setting);
        } else {
          context.update_thin_lines(
            name,
            reinterpret_cast<const Eigen::Vector4f*>(vertices.data()),
            reinterpret_cast<const Eigen::Vector4f*>(colors.data()),
            vertices.rows(),
            indices.data(),
            indices.size(),
            line_strip,
            shader_setting);
        }
      },
      py::arg("name"),
      py::arg("vertices"),
      py::arg("colors") = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>(),
      py::arg("indices") = std::vector<unsigned int>(),
      py::arg("line_strip") = false,
      py::arg("shader_setting") = guik::ShaderSetting(),
      R"doc(Register or update a thin lines drawable.

Args:
    name: Drawable name.
    vertices: Vertex positions (N x 3 or N x 4 array).
    colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
    indices: Vertex indices. If empty, non-indexed drawing is used.
    line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
    shader_setting: Shader setting.
)doc")

    .def(
      "update_thin_lines",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& vertices,
        const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& colors,
        const std::vector<unsigned int>& indices,
        bool line_strip,
        float line_width,
        const guik::ShaderSetting& shader_setting) {
        if (vertices.cols() != 3 && vertices.cols() != 4) {
          std::cerr << "warning: vertices must be Nx3 or Nx4" << std::endl;
          return;
        }
        if (colors.size()) {
          if (colors.rows() != vertices.rows()) {
            std::cerr << "warning: colors must have the same number of rows as vertices" << std::endl;
            return;
          }
          if (colors.cols() != 4) {
            std::cerr << "warning: colors must be Nx4" << std::endl;
            return;
          }
        }

        const unsigned int* index_ptr = indices.empty() ? nullptr : indices.data();

        if (vertices.cols() == 3) {
          const float* color_ptr = colors.size() ? colors.data() : nullptr;
          context.update_thin_lines(name, vertices.data(), color_ptr, vertices.rows(), index_ptr, indices.size(), line_strip, line_width, shader_setting);
        } else {
          const Eigen::Vector4f* color_ptr = colors.size() ? reinterpret_cast<const Eigen::Vector4f*>(colors.data()) : nullptr;
          context.update_thin_lines(
            name,
            reinterpret_cast<const Eigen::Vector4f*>(vertices.data()),
            color_ptr,
            vertices.rows(),
            index_ptr,
            indices.size(),
            line_strip,
            line_width,
            shader_setting);
        }
      },
      py::arg("name"),
      py::arg("vertices"),
      py::arg("colors") = Eigen::Matrix<float, -1, -1, Eigen::RowMajor>(),
      py::arg("indices") = std::vector<unsigned int>(),
      py::arg("line_strip") = false,
      py::arg("line_width") = 1.0f,
      py::arg("shader_setting") = guik::ShaderSetting(),
      R"doc(Register or update a thin lines drawable.

Args:
    name: Drawable name.
    vertices: Vertex positions (N x 3 or N x 4 array).
    colors: Vertex colors (N x 4 array). If empty, no vertex colors are assigned.
    indices: Vertex indices. If empty, non-indexed drawing is used.
    line_strip: If True, line strip mode is used. Otherwise, line list mode is used.
    line_width: Width of the lines in pixels.
    shader_setting: Shader setting.
)doc")

    .def("update_icosahedron", &guik::AsyncLightViewerContext::update_icosahedron, py::arg("name"), py::arg("shader_setting"), "Register or update an icosahedron primitive.")
    .def("update_sphere", &guik::AsyncLightViewerContext::update_sphere, py::arg("name"), py::arg("shader_setting"), "Register or update a sphere primitive.")
    .def("update_cube", &guik::AsyncLightViewerContext::update_cube, py::arg("name"), py::arg("shader_setting"), "Register or update a cube primitive.")
    .def("update_cone", &guik::AsyncLightViewerContext::update_cone, py::arg("name"), py::arg("shader_setting"), "Register or update a cone primitive.")
    .def("update_frustum", &guik::AsyncLightViewerContext::update_frustum, py::arg("name"), py::arg("shader_setting"), "Register or update a frustum primitive.")
    .def("update_coord", &guik::AsyncLightViewerContext::update_coord, py::arg("name"), py::arg("shader_setting"), "Register or update a coordinate axes primitive.")

    .def(
      "update_wire_icosahedron",
      &guik::AsyncLightViewerContext::update_wire_icosahedron,
      py::arg("name"),
      py::arg("shader_setting"),
      "Register or update a wireframe icosahedron primitive.")
    .def("update_wire_sphere", &guik::AsyncLightViewerContext::update_wire_sphere, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe sphere primitive.")
    .def("update_wire_cube", &guik::AsyncLightViewerContext::update_wire_cube, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe cube primitive.")
    .def("update_wire_cone", &guik::AsyncLightViewerContext::update_wire_cone, py::arg("name"), py::arg("shader_setting"), "Register or update a wireframe cone primitive.")
    .def(
      "update_wire_frustum",
      &guik::AsyncLightViewerContext::update_wire_frustum,
      py::arg("name"),
      py::arg("shader_setting"),
      "Register or update a wireframe frustum primitive.");

  // AsyncLightViewer
  py::class_<guik::AsyncLightViewer, guik::AsyncLightViewerContext, std::unique_ptr<guik::AsyncLightViewer, py::nodelete>>(guik_, "AsyncLightViewer", py::multiple_inheritance())
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background, const std::string& title) { return async_instance(size, background, title); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false,
      py::arg("title") = "screen",
      R"doc(Get the singleton instance of AsyncLightViewer. The viewer is created and run in a background thread upon the first call.

Args:
    size: Initial canvas size. Ignored if the viewer has already been created.
    background: If True, the viewer window is created in the background (invisible).
    title: Window title.
)doc")

    .def_static("destroy", &guik::AsyncLightViewer::destroy, "Destroy the viewer in the background thread.")
    .def_static("wait", &guik::AsyncLightViewer::wait, "Wait for the viewer to be closed.")
    .def_static("wait_until_click", &guik::AsyncLightViewer::wait_until_click, "Wait for a break button to be clicked.")
    .def_static("toggle_wait", &guik::AsyncLightViewer::toggle_wait, "Wait while a toggle checkbox is checked.")

    .def("clear_images", &guik::AsyncLightViewer::clear_images, "Clear all images.")
    .def("remove_image", &guik::AsyncLightViewer::remove_image, py::arg("name"), "Remove an image by name.")
    .def(
      "update_image",
      &guik::AsyncLightViewer::update_image,
      py::arg("name"),
      py::arg("width"),
      py::arg("height"),
      py::arg("rgba_bytes"),
      py::arg("scale") = -1.0,
      py::arg("order") = -1,
      R"doc(Register or update an image.

Args:
    name: Name of the image.
    width: Image width.
    height: Image height.
    rgba_bytes: Image data in RGBA format (width * height * 4 bytes).
    scale: Scale factor for displaying the image. If negative, auto scale is used.
    order: Display order. Images with smaller order values are displayed first. If -1, the image is added to the end of the display order.
)doc")

    .def(
      "async_sub_viewer",
      [](guik::AsyncLightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.async_sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      },
      py::arg("name"),
      py::arg("size") = std::make_tuple(-1, -1),
      R"doc(Create or get an async sub viewer context.

Args:
    name: Name of the sub viewer context.
    size: Canvas size of the sub viewer. If (-1, -1), the default size is used.
)doc")

    .def("remove_async_sub_viewer", &guik::AsyncLightViewer::remove_async_sub_viewer, py::arg("name"), "Remove an async sub viewer context by name.")

    .def(
      "update_plot_line",
      [](
        guik::AsyncLightViewer& viewer,
        const std::string& plot_name,
        const std::string& label,
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        int line_flags,
        int max_num_data) {
        if (ys.empty()) {
          viewer.update_plot_line(plot_name, label, xs, line_flags, max_num_data);
        } else {
          viewer.update_plot_line(plot_name, label, xs, ys, line_flags, max_num_data);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("line_flags") = 0,
      py::arg("max_num_data") = 8192 * 128,
      R"doc(Update a line plot.

Args:
    plot_name: Name of the plot.
    label: Label.
    xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
    ys: Y data.
    line_flags: Flags for the line plot (ImPlotLineFlags).
    max_num_data: Maximum number of data points to be displayed.
)doc")

    .def(
      "update_plot_scatter",
      [](guik::AsyncLightViewer& viewer, const std::string& plot_name, const std::string& label, const std::vector<double>& xs, const std::vector<double>& ys, int scatter_flags) {
        if (ys.empty()) {
          viewer.update_plot_scatter(plot_name, label, xs, scatter_flags);
        } else {
          viewer.update_plot_scatter(plot_name, label, xs, ys, scatter_flags);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("scatter_flags") = 0,
      R"doc(Update a scatter plot.

Args:
    plot_name: Name of the plot.
    label: Label.
    xs: X data. If ys is empty, xs is used as Y data (X = 0, 1, 2, ...).
    ys: Y data.
    scatter_flags: Flags for the scatter plot (ImPlotScatterFlags).
)doc")

    .def(
      "update_plot_histogram",
      [](
        guik::AsyncLightViewer& viewer,
        const std::string& plot_name,
        const std::string& label,
        const std::vector<double>& xs,
        const std::vector<double>& ys,
        int x_bins,
        int y_bins,
        const Eigen::Vector2d& x_range,
        const Eigen::Vector2d& y_range,
        int histogram_flags) {
        if (ys.empty()) {
          viewer.update_plot_histogram(plot_name, label, xs, x_bins, x_range, histogram_flags);
        } else {
          viewer.update_plot_histogram(plot_name, label, xs, ys, x_bins, y_bins, x_range, y_range, histogram_flags);
        }
      },
      py::arg("plot_name"),
      py::arg("label"),
      py::arg("xs"),
      py::arg("ys") = std::vector<double>(),
      py::arg("x_bins") = -2,
      py::arg("y_bins") = -2,
      py::arg("x_range") = Eigen::Vector2d(0.0, 0.0),
      py::arg("y_range") = Eigen::Vector2d(0.0, 0.0),
      py::arg("histogram_flags") = 0,
      R"doc(Update a histogram plot. If ys is empty, a 1D histogram of xs is plotted. Otherwise, a 2D histogram of (xs, ys) is plotted.

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
)doc")

    .def("fit_plot", &guik::AsyncLightViewer::fit_plot, py::arg("plot_name"), "Fit the plot view to the data range.")
    .def("fit_all_plots", &guik::AsyncLightViewer::fit_all_plots, "Fit all plots to their data ranges.")

    ;

  // guik::RecentFiles
  py::class_<guik::RecentFiles>(guik_, "RecentFiles", "History of recently opened files, persisted per tag.")
    .def(py::init<const std::string&>(), py::arg("tag"))
    .def("clear", &guik::RecentFiles::clear)
    .def(
      "push",
      [](guik::RecentFiles& recent, const std::string& filename) { recent.push(filename); },
      py::arg("filename"))
    .def(
      "push_all",
      [](guik::RecentFiles& recent, const std::vector<std::string>& filenames) { recent.push(filenames); },
      py::arg("filenames"))
    .def("empty", &guik::RecentFiles::empty)
    .def("size", &guik::RecentFiles::size)
    .def("most_recent", &guik::RecentFiles::most_recent)
    .def("fullpath", &guik::RecentFiles::fullpath, py::arg("i"))
    .def("filename", &guik::RecentFiles::filename, py::arg("i"));

  // Global methods
  guik_.def("anon", &guik::anon, "Generate a unique name.");
  guik_.def(
    "viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen",
    R"doc(Get the singleton LightViewer instance. The viewer instance is created upon the first call.

Args:
    size: Initial canvas size. Ignored if the viewer has already been created.
    background: If True, the viewer window is created in the background (invisible).
    title: Window title.
)doc");
  guik_.def("destroy", &guik::destroy, "Destroy the viewer");
  guik_.def("running", &guik::running, "Check if the viewer is currently running");
  guik_.def(
    "async_viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return async_instance(size, background, title); },
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen",
    R"doc(Get the singleton AsyncLightViewer instance. The viewer is created and run in a background thread upon the first call.

Args:
    size: Initial canvas size. Ignored if the viewer has already been created.
    background: If True, the viewer window is created in the background (invisible).
    title: Window title.
)doc");
  guik_.def("async_destroy", &guik::async_destroy, "Destroy the async viewer");
  guik_.def("async_wait", &guik::async_wait, "Wait for the async viewer to be closed.");
  guik_.def("async_running", &guik::async_running, "Check if the async viewer is currently running");

  auto atexit = py::module_::import("atexit");
  atexit.attr("register")(py::cpp_function([]() {
    guik::async_destroy();
    guik::destroy();
  }));
}
