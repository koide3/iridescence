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
#include <guik/recent_files.hpp>
#include <guik/model_control.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/arcball_camera_control.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/camera/fps_camera_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/viewer/async_light_viewer.hpp>

namespace py = pybind11;

static bool is_first = true;

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
  py::enum_<guik::PointShapeMode::MODE>(guik_, "PointShapeMode")
    .value("RECTANGLE", guik::PointShapeMode::MODE::RECTANGLE)
    .value("CIRCLE", guik::PointShapeMode::MODE::CIRCLE)
    .export_values();

  // guik::PointScaleMode
  py::enum_<guik::PointScaleMode::MODE>(guik_, "PointScaleMode")
    .value("SCREENSPACE", guik::PointScaleMode::MODE::SCREENSPACE)
    .value("METRIC", guik::PointScaleMode::MODE::METRIC)
    .export_values();

  // classess
  // guik::ShaderSetting
  py::class_<guik::ShaderSetting, std::shared_ptr<guik::ShaderSetting>>(guik_, "ShaderSetting")
    .def("add", &guik::ShaderSetting::add<float>)
    .def("add", &guik::ShaderSetting::add<Eigen::Vector2f>)
    .def("add", &guik::ShaderSetting::add<Eigen::Matrix4f>)
    .def("addi", &guik::ShaderSetting::add<Eigen::Vector4i>)
    .def("static_object", &guik::ShaderSetting::static_object, py::return_value_policy::reference_internal)
    .def("dynamic_object", &guik::ShaderSetting::dynamic_object, py::return_value_policy::reference_internal)
    .def("material_color", &guik::ShaderSetting::material_color)
    .def(
      "set_color",
      [&](guik::ShaderSetting& self, float r, float g, float b, float a) { return self.set_color(Eigen::Vector4f(r, g, b, a)); },
      py::arg("r"),
      py::arg("g"),
      py::arg("b"),
      py::arg("a") = 1.0,
      py::return_value_policy::reference_internal)
    .def(
      "set_color",
      [&](guik::ShaderSetting& self, const Eigen::Vector4f& color) { return self.set_color(color); },
      py::arg("color"),
      py::return_value_policy::reference_internal)
    .def("set_alpha", &guik::ShaderSetting::set_alpha, py::return_value_policy::reference_internal)
    .def("make_transparent", &guik::ShaderSetting::make_transparent, py::return_value_policy::reference_internal)
    .def("point_scale", &guik::ShaderSetting::point_scale)
    .def("set_point_scale", &guik::ShaderSetting::set_point_scale, py::arg("scale"), py::return_value_policy::reference_internal)
    .def("set_point_size", &guik::ShaderSetting::set_point_size, py::arg("size"), py::return_value_policy::reference_internal)
    .def("set_point_size_offset", &guik::ShaderSetting::set_point_size_offset, py::arg("offset"), py::return_value_policy::reference_internal)
    .def("set_point_shape_mode", &guik::ShaderSetting::set_point_shape_mode, py::arg("mode"), py::return_value_policy::reference_internal)
    .def("set_point_shape_circle", &guik::ShaderSetting::set_point_shape_circle, py::return_value_policy::reference_internal)
    .def("set_point_shape_rectangle", &guik::ShaderSetting::set_point_shape_rectangle, py::return_value_policy::reference_internal)
    .def("set_point_scale_mode", &guik::ShaderSetting::set_point_scale_mode, py::arg("mode"), py::return_value_policy::reference_internal)
    .def("set_point_scale_screenspace", &guik::ShaderSetting::set_point_scale_screenspace, py::return_value_policy::reference_internal)
    .def("set_point_scale_metric", &guik::ShaderSetting::set_point_scale_metric, py::return_value_policy::reference_internal)
    .def("set_point_shape", &guik::ShaderSetting::set_point_shape, py::arg("point_size"), py::arg("metric"), py::arg("circle"), py::return_value_policy::reference_internal)
    .def("remove_model_matrix", &guik::ShaderSetting::remove_model_matrix, py::return_value_policy::reference_internal)
    .def("color_mode", &guik::ShaderSetting::color_mode)
    .def("set_color_mode", &guik::ShaderSetting::set_color_mode, py::arg("color_mode"), py::return_value_policy::reference_internal)
    .def("model_matrix", &guik::ShaderSetting::model_matrix)
    .def("translation", &guik::ShaderSetting::translation)
    .def("rotation", &guik::ShaderSetting::rotation)
    .def("set_model_matrix", &guik::ShaderSetting::set_model_matrix<Eigen::Matrix4f>, py::arg("model_matrix"), py::return_value_policy::reference_internal)
    .def("transform", &guik::ShaderSetting::transform<Eigen::Matrix4f>)
    .def(
      "translate",
      [](guik::ShaderSetting& self, float x, float y, float z) { return self.translate(x, y, z); },
      py::arg("x"),
      py::arg("y"),
      py::arg("z"),
      py::return_value_policy::reference_internal)
    .def(
      "translate",
      [](guik::ShaderSetting& self, const Eigen::Vector3f& translation) { return self.translate(translation); },
      py::arg("translation"),
      py::return_value_policy::reference_internal)
    .def(
      "rotate",
      [](guik::ShaderSetting& self, float angle, const Eigen::Vector3f& axis) { return self.rotate(angle, axis); },
      py::arg("angle"),
      py::arg("axis"),
      py::return_value_policy::reference_internal)
    .def(
      "rotate",
      [](guik::ShaderSetting& self, const Eigen::Matrix3f& rotation) { return self.rotate(rotation); },
      py::arg("rotation"),
      py::return_value_policy::reference_internal)
    .def(
      "scale",
      [](guik::ShaderSetting& self, float scale) { return self.scale(scale); },
      py::arg("scale"),
      py::return_value_policy::reference_internal)
    .def("scale", [](guik::ShaderSetting& self, const Eigen::Vector3f& scale) { return self.scale(scale); }, py::arg("scale"), py::return_value_policy::reference_internal);

  // guik::Rainbow
  py::class_<guik::Rainbow, guik::ShaderSetting, std::shared_ptr<guik::Rainbow>>(guik_, "Rainbow")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
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
  py::class_<guik::VertexColor, guik::ShaderSetting, std::shared_ptr<guik::VertexColor>>(guik_, "VertexColor")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
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

  // guik::FlatColor
  py::class_<guik::FlatColor, guik::ShaderSetting, std::shared_ptr<guik::FlatColor>>(guik_, "FlatColor")
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
  py::class_<guik::FlatRed, guik::FlatColor, std::shared_ptr<guik::FlatRed>>(guik_, "FlatRed")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::FlatRed(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  py::class_<guik::FlatGreen, guik::FlatColor, std::shared_ptr<guik::FlatGreen>>(guik_, "FlatGreen")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::FlatGreen(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  py::class_<guik::FlatBlue, guik::FlatColor, std::shared_ptr<guik::FlatBlue>>(guik_, "FlatBlue")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::FlatBlue(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  py::class_<guik::FlatOrange, guik::FlatColor, std::shared_ptr<guik::FlatOrange>>(guik_, "FlatOrange")
    .def(py::init<>())
    .def(py::init<Eigen::Matrix4f>())
    .def(
      py::init([](float scale, const Eigen::Vector3f& trans, const Eigen::Matrix3f& rot) {
        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3, 3>(0, 0) = scale * rot;
        mat.block<3, 1>(0, 3) = trans;
        return new guik::FlatOrange(mat);
      }),
      py::arg("scale") = 1.0,
      py::arg("trans") = Eigen::Vector3f::Zero(),
      py::arg("rot") = Eigen::Matrix3f::Identity());

  // guik::ModelControl
  py::class_<guik::ModelControl>(guik_, "ModelControl")
    .def(py::init<std::string, Eigen::Matrix4f>(), py::arg("label") = "model_control", py::arg("model_matrix") = Eigen::Matrix4f::Identity())
    .def("draw_ui", &guik::ModelControl::draw_ui)
    .def("draw_gizmo_ui", &guik::ModelControl::draw_gizmo_ui)
    .def(
      "draw_gizmo",
      [](guik::ModelControl& control) {
        auto viewer = guik::LightViewer::instance();
        control.draw_gizmo(0, 0, viewer->canvas_size().x(), viewer->canvas_size().y(), viewer->view_matrix(), viewer->projection_matrix());
      })
    .def("model_matrix", &guik::ModelControl::model_matrix)
    .def(
      "set_model_matrix",
      [](guik::ModelControl& model_control, const Eigen::Matrix4f& mat) { model_control.set_model_matrix(mat); },
      py::arg("model_matrix"))
    .def("set_gizmo_enabled", &guik::ModelControl::set_gizmo_enabled)
    .def("enable_gizmo", &guik::ModelControl::enable_gizmo)
    .def("disable_gizmo", &guik::ModelControl::disable_gizmo)
    .def("set_gizmo_operation", [](guik::ModelControl& model_control, const std::string& op) { model_control.set_gizmo_operation(op); })
    .def("set_gizmo_mode", &guik::ModelControl::set_gizmo_mode)
    .def("set_gizmo_clip_scale", &guik::ModelControl::set_gizmo_clip_scale);

  // guik::CameraControl
  py::class_<guik::CameraControl, std::shared_ptr<guik::CameraControl>>(guik_, "CameraControl")
    .def("reset_center", &guik::CameraControl::reset_center)
    .def("lookat", &guik::CameraControl::lookat)
    .def("view_matrix", &guik::CameraControl::view_matrix);
  py::class_<guik::OrbitCameraControlXY, guik::CameraControl, std::shared_ptr<guik::OrbitCameraControlXY>>(guik_, "OrbitCameraControlXY");
  py::class_<guik::OrbitCameraControlXZ, guik::CameraControl, std::shared_ptr<guik::OrbitCameraControlXZ>>(guik_, "OrbitCameraControlXZ");
  py::class_<guik::ArcBallCameraControl, guik::CameraControl, std::shared_ptr<guik::ArcBallCameraControl>>(guik_, "ArcBallCameraControl");
  py::class_<guik::TopDownCameraControl, guik::CameraControl, std::shared_ptr<guik::TopDownCameraControl>>(guik_, "TopDownCameraControl");
  py::class_<guik::FPSCameraControl, guik::CameraControl, std::shared_ptr<guik::FPSCameraControl>>(guik_, "FPSCameraControl")
    .def("set_fovy", &guik::FPSCameraControl::set_fovy)
    .def("set_depth_range", &guik::FPSCameraControl::set_depth_range)
    .def("set_pose", &guik::FPSCameraControl::set_pose)
    .def("set_mouse_senstivity", &guik::FPSCameraControl::set_mouse_senstivity)
    .def("set_translation_speed", &guik::FPSCameraControl::set_translation_speed);

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
    .def("clear", &guik::HoveredDrawings::clear)
    .def("create_callback", &guik::HoveredDrawings::create_callback);

  // guik::ProjectionControl
  py::class_<guik::ProjectionControl, std::shared_ptr<guik::ProjectionControl>>(guik_, "ProjectionControl");

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
    .def("resize", &guik::Application::resize)
    .def("set_title", &guik::Application::set_title)
    .def("framebuffer_size", &guik::Application::framebuffer_size)
    .def("spin", &guik::Application::spin)
    .def("spin_once", &guik::Application::spin_once)
    .def("close", &guik::Application::close)
    .def("closed", &guik::Application::closed);

  // LightViewerContext
  py::class_<guik::LightViewerContext, std::unique_ptr<guik::LightViewerContext, py::nodelete>>(guik_, "LightViewerContext")
    .def("set_size", &guik::LightViewerContext::set_size)
    .def("set_clear_color", &guik::LightViewerContext::set_clear_color)

    .def("set_pos", &guik::LightViewerContext::set_pos, "", py::arg("pos"), py::arg("cond") = static_cast<int>(ImGuiCond_FirstUseEver), py::arg("flags") = 0)
    .def("show", &guik::LightViewerContext::show)
    .def("hide", &guik::LightViewerContext::hide)

    .def("clear", &guik::LightViewerContext::clear)
    .def("clear_text", &guik::LightViewerContext::clear_text)
    .def("append_text", &guik::LightViewerContext::append_text)
    .def("register_ui_callback", &guik::LightViewerContext::register_ui_callback, py::arg("callback_name"), py::arg("callback"))
    .def("remove_ui_callback", &guik::LightViewerContext::remove_ui_callback, py::arg("callback_name"))

    .def(
      "shader_setting",
      [](guik::LightViewerContext& context) { return &context.shader_setting(); },
      py::return_value_policy::reference)

    .def("disable_xy_grid", &guik::LightViewerContext::disable_xy_grid)
    .def("enable_xy_grid", &guik::LightViewerContext::enable_xy_grid)
    .def("set_draw_xy_grid", &guik::LightViewerContext::set_draw_xy_grid)
    .def("set_colormap", &guik::LightViewerContext::set_colormap)
    .def("set_screen_effect", &guik::LightViewerContext::set_screen_effect)
    .def("get_screen_effect", &guik::LightViewerContext::get_screen_effect)
    .def("set_bg_texture", &guik::LightViewerContext::set_bg_texture)

    .def("enable_normal_buffer", &guik::LightViewerContext::enable_normal_buffer)
    .def("enable_info_buffer", &guik::LightViewerContext::enable_info_buffer)
    .def("enable_partial_rendering", &guik::LightViewerContext::enable_partial_rendering, py::arg("clear_thresh") = 1e-6)
    .def("disable_partial_rendering", &guik::LightViewerContext::disable_partial_rendering)

    .def("color_buffer", &guik::LightViewerContext::color_buffer)
    .def("depth_buffer", &guik::LightViewerContext::depth_buffer)
    .def("normal_buffer", &guik::LightViewerContext::normal_buffer)
    .def("info_buffer", &guik::LightViewerContext::info_buffer)

    .def("clear_drawables", [](guik::LightViewerContext& context) { context.clear_drawables(); })

    .def("get_drawables", &guik::LightViewerContext::get_drawables)
    .def("find_drawable", &guik::LightViewerContext::find_drawable)
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
      py::arg("regex") = false)
    .def("update_drawable", &guik::LightViewerContext::update_drawable)

    .def("clear_drawable_filters", &guik::LightViewerContext::clear_drawable_filters)
    .def("register_drawable_filter", &guik::LightViewerContext::register_drawable_filter)
    .def("remove_drawable_filter", &guik::LightViewerContext::remove_drawable_filter)

    .def("clear_partial_rendering", &guik::LightViewerContext::clear_partial_rendering)

    .def("get_camera_control", &guik::LightViewerContext::get_camera_control)
    .def("set_camera_control", &guik::LightViewerContext::set_camera_control)
    .def("get_projection_control", &guik::LightViewerContext::get_projection_control)
    .def("set_projection_control", &guik::LightViewerContext::set_projection_control)

    .def("save_camera_settings", &guik::LightViewerContext::save_camera_settings, py::arg("path"))
    .def("load_camera_settings", &guik::LightViewerContext::load_camera_settings, py::arg("path"))

    .def("save_color_buffer", &guik::LightViewerContext::save_color_buffer, py::arg("path"))
    .def("save_depth_buffer", &guik::LightViewerContext::save_depth_buffer, py::arg("path"), py::arg("real_scale") = true)

    .def("reset_center", &guik::LightViewerContext::reset_center)
    .def(
      "lookat",
      [](guik::LightViewerContext& context, const Eigen::Vector3f& pt) { context.lookat(pt); },
      py::arg("pt"))
    .def(
      "use_orbit_camera_control",
      &guik::LightViewerContext::use_orbit_camera_control,
      "",
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_orbit_camera_control_xz", &guik::LightViewerContext::use_orbit_camera_control_xz, "", py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = 0.0)
    .def("use_topdown_camera_control", &guik::LightViewerContext::use_topdown_camera_control, "", py::arg("distance") = 80.0, py::arg("theta") = 0.0)
    .def("use_fps_camera_control", &guik::LightViewerContext::use_fps_camera_control, "", py::arg("fovy_deg") = 60.0)
    .def(
      "use_arcball_camera_control",
      &guik::LightViewerContext::use_arcball_camera_control,
      "",
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_fps_camera_control", &guik::LightViewerContext::use_fps_camera_control, py::arg("fovy_deg") = 60.0)

    .def("set_point_shape", &guik::LightViewerContext::set_point_shape, py::arg("point_size") = 1.0f, py::arg("metric") = true, py::arg("circle") = true)

    .def("pick_info", &guik::LightViewerContext::pick_info, py::arg("p"), py::arg("window") = 2)
    .def("pick_depth", &guik::LightViewerContext::pick_depth, py::arg("p"), py::arg("window") = 2)
    .def("unproject", &guik::LightViewerContext::unproject, py::arg("p"), py::arg("depth"))

    .def(
      "update_points",
      [](guik::LightViewerContext& context, const std::string& name, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points, const guik::ShaderSetting& shader_setting)
        -> std::shared_ptr<glk::PointCloudBuffer> {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return nullptr;
        }
        return context.update_points(name, points.data(), sizeof(float) * points.cols(), points.rows(), shader_setting);
      })

    .def(
      "update_normal_dists",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Matrix3f>& covs,
        float scale,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
      })

    .def(
      "update_normal_dists",
      [](
        guik::LightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector4f>& points,
        const std::vector<Eigen::Matrix4f>& covs,
        float scale,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
      })

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
      py::arg("shader_setting") = guik::ShaderSetting())

    .def("update_icosahedron", &guik::LightViewerContext::update_icosahedron)
    .def("update_sphere", &guik::LightViewerContext::update_sphere)
    .def("update_cube", &guik::LightViewerContext::update_cube)
    .def("update_cone", &guik::LightViewerContext::update_cone)
    .def("update_frustum", &guik::LightViewerContext::update_frustum)
    .def("update_coord", &guik::LightViewerContext::update_coord)

    .def("update_wire_icosahedron", &guik::LightViewerContext::update_wire_icosahedron)
    .def("update_wire_sphere", &guik::LightViewerContext::update_wire_sphere)
    .def("update_wire_cube", &guik::LightViewerContext::update_wire_cube)
    .def("update_wire_cone", &guik::LightViewerContext::update_wire_cone)
    .def("update_wire_frustum", &guik::LightViewerContext::update_wire_frustum)

    ;

  // LightViewer
  py::class_<guik::LightViewer, guik::Application, guik::LightViewerContext, std::unique_ptr<guik::LightViewer, py::nodelete>>(guik_, "LightViewer")
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false,
      py::arg("title") = "screen")
    .def(
      "sub_viewer",
      [](guik::LightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      },
      py::arg("name"),
      py::arg("size") = std::make_tuple(-1, -1))

    .def("spin_until_click", &guik::LightViewer::spin_until_click)
    .def("toggle_spin_once", &guik::LightViewer::toggle_spin_once)

    .def("clear_images", &guik::LightViewer::clear_images)
    .def("remove_image", &guik::LightViewer::remove_image)
    .def("update_image", &guik::LightViewer::update_image, py::arg("name"), py::arg("image"), py::arg("scale") = -1.0, py::arg("order") = -1)

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
      py::arg("max_num_data") = 8192 * 128)

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
      py::arg("scatter_flats") = 0)

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
      py::arg("histogram_flags") = 0)

    .def("fit_plot", &guik::LightViewer::fit_plot, py::arg("plot_name"))
    .def("fit_all_plots", &guik::LightViewer::fit_all_plots)

    ;

  // AsyncLightViewerContext
  py::class_<guik::AsyncLightViewerContext, std::unique_ptr<guik::AsyncLightViewerContext, py::nodelete>>(guik_, "AsyncLightViewerContext")
    .def("clear", &guik::AsyncLightViewerContext::clear)
    .def("clear_text", &guik::AsyncLightViewerContext::clear_text)
    .def("append_text", &guik::AsyncLightViewerContext::append_text)
    .def("register_ui_callback", &guik::AsyncLightViewerContext::register_ui_callback, py::arg("callback_name"), py::arg("callback"))
    .def("remove_ui_callback", &guik::AsyncLightViewerContext::remove_ui_callback, py::arg("callback_name"))

    .def("disable_xy_grid", &guik::AsyncLightViewerContext::disable_xy_grid)
    .def("enable_xy_grid", &guik::AsyncLightViewerContext::enable_xy_grid)
    .def("set_draw_xy_grid", &guik::AsyncLightViewerContext::set_draw_xy_grid)
    .def("set_colormap", &guik::AsyncLightViewerContext::set_colormap)

    .def("set_point_shape", &guik::AsyncLightViewerContext::set_point_shape, py::arg("point_size") = 1.0f, py::arg("metric") = true, py::arg("circle") = true)

    .def("clear_drawables", [](guik::AsyncLightViewerContext& context) { context.clear_drawables(); })
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
      py::arg("regex") = false)

    .def("save_camera_settings", &guik::AsyncLightViewerContext::save_camera_settings, py::arg("path"))
    .def("load_camera_settings", &guik::AsyncLightViewerContext::load_camera_settings, py::arg("path"))
    .def("save_color_buffer", &guik::AsyncLightViewerContext::save_color_buffer, py::arg("path"))
    .def("save_depth_buffer", &guik::AsyncLightViewerContext::save_depth_buffer, py::arg("path"), py::arg("real_scale") = true)

    .def("reset_center", &guik::AsyncLightViewerContext::reset_center)
    .def(
      "lookat",
      [](guik::AsyncLightViewerContext& context, const Eigen::Vector3f& pt) { context.lookat(pt); },
      py::arg("pt"))

    .def(
      "use_orbit_camera_control",
      &guik::AsyncLightViewerContext::use_orbit_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_orbit_camera_control_xz", &guik::AsyncLightViewerContext::use_orbit_camera_control_xz, py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = 0.0)
    .def("use_topdown_camera_control", &guik::AsyncLightViewerContext::use_topdown_camera_control, py::arg("distance") = 80.0, py::arg("theta") = 0.0)
    .def(
      "use_arcball_camera_control",
      &guik::AsyncLightViewerContext::use_arcball_camera_control,
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_fps_camera_control", &guik::AsyncLightViewerContext::use_fps_camera_control, py::arg("fovy_deg") = 60.0)

    .def("update_drawable_setting", &guik::AsyncLightViewerContext::update_drawable_setting, py::arg("name"), py::arg("setting"))

    .def(
      "update_points",
      [](guik::AsyncLightViewerContext& context, const std::string& name, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points, const guik::ShaderSetting& shader_setting) {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return;
        }
        context.update_points(name, points.data(), sizeof(float) * points.cols(), points.rows(), shader_setting);
      })

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
      })
    .def(
      "update_normal_dists",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<Eigen::Matrix3f>& covs,
        float scale,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
      })

    .def(
      "update_normal_dists",
      [](
        guik::AsyncLightViewerContext& context,
        const std::string& name,
        const std::vector<Eigen::Vector4f>& points,
        const std::vector<Eigen::Matrix4f>& covs,
        float scale,
        const guik::ShaderSetting& shader_setting) {
        if (covs.size() != points.size()) {
          std::cerr << "warning: covs must have the same number of rows as points" << std::endl;
          return;
        }

        context.update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
      })

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
      py::arg("shader_setting") = guik::ShaderSetting())

    .def("update_icosahedron", &guik::AsyncLightViewerContext::update_icosahedron)
    .def("update_sphere", &guik::AsyncLightViewerContext::update_sphere)
    .def("update_cube", &guik::AsyncLightViewerContext::update_cube)
    .def("update_cone", &guik::AsyncLightViewerContext::update_cone)
    .def("update_frustum", &guik::AsyncLightViewerContext::update_frustum)
    .def("update_coord", &guik::AsyncLightViewerContext::update_coord)

    .def("update_wire_icosahedron", &guik::AsyncLightViewerContext::update_wire_icosahedron)
    .def("update_wire_sphere", &guik::AsyncLightViewerContext::update_wire_sphere)
    .def("update_wire_cube", &guik::AsyncLightViewerContext::update_wire_cube)
    .def("update_wire_cone", &guik::AsyncLightViewerContext::update_wire_cone)
    .def("update_wire_frustum", &guik::AsyncLightViewerContext::update_wire_frustum);

  // AsyncLightViewer
  py::class_<guik::AsyncLightViewer, guik::AsyncLightViewerContext, std::unique_ptr<guik::AsyncLightViewer, py::nodelete>>(guik_, "AsyncLightViewer", py::multiple_inheritance())
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false,
      py::arg("title") = "screen")

    .def_static("destroy", &guik::AsyncLightViewer::destroy)
    .def_static("wait", &guik::AsyncLightViewer::wait)
    .def_static("wait_until_click", &guik::AsyncLightViewer::wait_until_click)
    .def_static("toggle_wait", &guik::AsyncLightViewer::toggle_wait)

    .def("clear_images", &guik::AsyncLightViewer::clear_images)
    .def("remove_image", &guik::AsyncLightViewer::remove_image)
    .def(
      "update_image",
      &guik::AsyncLightViewer::update_image,
      py::arg("name"),
      py::arg("width"),
      py::arg("height"),
      py::arg("rgba_bytes"),
      py::arg("scale") = -1.0,
      py::arg("order") = -1)

    .def(
      "async_sub_viewer",
      [](guik::AsyncLightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.async_sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      })

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
      py::arg("max_num_data") = 8192 * 128)

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
      py::arg("scatter_flats") = 0)

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
      py::arg("histogram_flags") = 0)

    .def("fit_plot", &guik::AsyncLightViewer::fit_plot, py::arg("plot_name"))
    .def("fit_all_plots", &guik::AsyncLightViewer::fit_all_plots)

    ;

  // guik::RecentFiles
  py::class_<guik::RecentFiles>(guik_, "RecentFiles")
    .def(py::init<const std::string&>())
    .def("clear", &guik::RecentFiles::clear)
    .def("push", [](guik::RecentFiles& recent, const std::string& filename) { recent.push(filename); })
    .def("push_all", [](guik::RecentFiles& recent, const std::vector<std::string>& filenames) { recent.push(filenames); })
    .def("empty", &guik::RecentFiles::empty)
    .def("size", &guik::RecentFiles::size)
    .def("most_recent", &guik::RecentFiles::most_recent)
    .def("fullpath", &guik::RecentFiles::fullpath)
    .def("filename", &guik::RecentFiles::filename);

  // Global methods
  guik_.def("anon", &guik::anon);
  guik_.def(
    "viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen");
  guik_.def("destroy", &guik::destroy, "");
  guik_.def(
    "async_viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return async_instance(size, background, title); },
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen");
  guik_.def("async_destroy", &guik::async_destroy, "");
  guik_.def("async_wait", &guik::async_wait, "");

  auto atexit = py::module_::import("atexit");
  atexit.attr("register")(py::cpp_function([]() {
    guik::async_destroy();
    guik::destroy();
  }));
}
