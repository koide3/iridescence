#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <filesystem>

#include <glk/path.hpp>
#include <glk/texture.hpp>
#include <guik/recent_files.hpp>
#include <guik/model_control.hpp>
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
    py::object pyridescence = py::module::import("pyridescence");
    std::filesystem::path path(pyridescence.attr("__file__").cast<std::string>());

    glk::set_data_path(path.parent_path().string() + "/data");
    is_first = false;
  }

  return guik::AsyncLightViewer::instance(size, background, title);
}

void define_guik(py::module_& m) {
  py::module_ guik_ = m.def_submodule("guik", "");

  // classess
  // guik::ShaderSetting
  py::class_<guik::ShaderSetting, std::shared_ptr<guik::ShaderSetting>>(guik_, "ShaderSetting")
    .def("add", &guik::ShaderSetting::add<float>)
    .def("add", &guik::ShaderSetting::add<Eigen::Vector2f>)
    .def("add", &guik::ShaderSetting::add<Eigen::Matrix4f>)
    .def("addi", &guik::ShaderSetting::add<Eigen::Vector4i>)
    .def("make_transparent", &guik::ShaderSetting::make_transparent);

  // guik::Rainbow
  py::class_<guik::Rainbow, guik::ShaderSetting, std::shared_ptr<guik::Rainbow>>(guik_, "Rainbow")
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
  py::class_<guik::CameraControl, std::shared_ptr<guik::CameraControl>>(guik_, "CameraControl");
  py::class_<guik::OrbitCameraControlXY, std::shared_ptr<guik::OrbitCameraControlXY>>(guik_, "OrbitCameraControlXY");
  py::class_<guik::OrbitCameraControlXZ, std::shared_ptr<guik::OrbitCameraControlXZ>>(guik_, "OrbitCameraControlXZ");
  py::class_<guik::ArcBallCameraControl, std::shared_ptr<guik::ArcBallCameraControl>>(guik_, "ArcBallCameraControl");
  py::class_<guik::TopDownCameraControl, std::shared_ptr<guik::TopDownCameraControl>>(guik_, "TopDownCameraControl");
  py::class_<guik::FPSCameraControl, std::shared_ptr<guik::FPSCameraControl>>(guik_, "FPSCameraControl")
    .def("set_fovy", &guik::FPSCameraControl::set_fovy)
    .def("set_depth_range", &guik::FPSCameraControl::set_depth_range)
    .def("set_pose", &guik::FPSCameraControl::set_pose)
    .def("set_mouse_senstivity", &guik::FPSCameraControl::set_mouse_senstivity)
    .def("set_translation_speed", &guik::FPSCameraControl::set_translation_speed);

  // guik::ProjectionControl
  py::class_<guik::ProjectionControl, std::shared_ptr<guik::ProjectionControl>>(guik_, "ProjectionControl");

  // LightViewerContext
  py::class_<guik::LightViewerContext, std::unique_ptr<guik::LightViewerContext, py::nodelete>>(guik_, "LightViewerContext")
    .def("set_size", &guik::LightViewerContext::set_size)
    .def("set_clear_color", &guik::LightViewerContext::set_clear_color)
    .def("set_pos", &guik::LightViewerContext::set_pos, "", py::arg("pos"), py::arg("cond") = static_cast<int>(ImGuiCond_FirstUseEver), py::arg("flags") = 0)
    .def("clear", &guik::LightViewerContext::clear)
    .def("clear_text", &guik::LightViewerContext::clear_text)
    .def("append_text", &guik::LightViewerContext::append_text)
    .def("register_ui_callback", &guik::LightViewerContext::register_ui_callback, py::arg("callback_name"), py::arg("callback"))
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
    .def("find_drawable", &guik::LightViewerContext::find_drawable)
    .def("reset_center", &guik::LightViewerContext::reset_center)
    .def("lookat", &guik::LightViewerContext::lookat)
    .def("set_draw_xy_grid", &guik::LightViewerContext::set_draw_xy_grid, "")
    .def("set_screen_effect", &guik::LightViewerContext::set_screen_effect, "")
    .def("enable_normal_buffer", &guik::LightViewerContext::enable_normal_buffer, "")
    .def("enable_info_buffer", &guik::LightViewerContext::enable_info_buffer, "")
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

    .def("pick_info", &guik::LightViewerContext::pick_info, py::arg("p"), py::arg("window") = 2)
    .def("pick_depth", &guik::LightViewerContext::pick_depth, py::arg("p"), py::arg("window") = 2)
    .def("unproject", &guik::LightViewerContext::unproject, py::arg("p"), py::arg("depth"))

    .def("get_camera_control", &guik::LightViewerContext::get_camera_control, "")
    .def("set_camera_control", &guik::LightViewerContext::set_camera_control, "")
    .def("get_projection_control", &guik::LightViewerContext::get_projection_control, "")
    .def("set_projection_control", &guik::LightViewerContext::set_projection_control, "");

  // LightViewer
  py::class_<guik::LightViewer, guik::LightViewerContext, std::unique_ptr<guik::LightViewer, py::nodelete>>(guik_, "LightViewer")
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

    .def("close", &guik::LightViewer::close)
    .def("spin", &guik::LightViewer::spin)
    .def("spin_once", &guik::LightViewer::spin_once)
    .def("toggle_spin_once", &guik::LightViewer::toggle_spin_once)
    .def("spin_until_click", &guik::LightViewer::spin_until_click)

    .def("enable_vsync", &guik::LightViewer::enable_vsync)
    .def("disable_vsync", &guik::LightViewer::disable_vsync)
    .def("enable_docking", &guik::LightViewer::enable_docking)

    .def("clear_images", &guik::LightViewer::clear_images)
    .def("remove_image", &guik::LightViewer::remove_image)
    .def("update_image", &guik::LightViewer::update_image, py::arg("name"), py::arg("image"), py::arg("scale") = -1.0, py::arg("order") = -1)

    .def(
      "update_points",
      [](guik::LightViewer& context, const std::string& name, const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>& points, const guik::ShaderSetting& shader_setting) {
        if (points.cols() != 3 && points.cols() != 4) {
          std::cerr << "warning: points must be Nx3 or Nx4" << std::endl;
          return;
        }
        context.update_points(name, points.data(), sizeof(float) * points.cols(), points.rows(), shader_setting);
      })

    // LightViewerContext methods
    .def("clear", &guik::LightViewer::clear)
    .def("clear_text", &guik::LightViewer::clear_text)
    .def("append_text", &guik::LightViewer::append_text, py::arg("text"))
    .def("register_ui_callback", &guik::LightViewer::register_ui_callback, py::arg("callback_name"), py::arg("callback"))

    .def(
      "remove_drawable",
      [](guik::LightViewer& context, const std::string& pattern, bool regex) {
        if (regex) {
          context.remove_drawable(std::regex(pattern));
        } else {
          context.remove_drawable(pattern);
        }
      },
      py::arg("pattern"),
      py::arg("regex") = false)
    .def("update_drawable", &guik::LightViewer::update_drawable, py::arg("name"), py::arg("drawable"), py::arg("settings"))
    .def("find_drawable", &guik::LightViewer::find_drawable, py::arg("name"))
    .def("reset_center", &guik::LightViewer::reset_center)
    .def("lookat", &guik::LightViewer::lookat, py::arg("pt"))
    .def("set_draw_xy_grid", &guik::LightViewer::set_draw_xy_grid, py::arg("enable"))
    .def("set_screen_effect", &guik::LightViewer::set_screen_effect, py::arg("effect"))
    .def("enable_normal_buffer", &guik::LightViewer::enable_normal_buffer)
    .def("enable_info_buffer", &guik::LightViewer::enable_info_buffer)
    .def("use_orbit_camera_control", &guik::LightViewer::use_orbit_camera_control, py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_orbit_camera_control_xz", &guik::LightViewer::use_orbit_camera_control_xz, py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = 0.0)
    .def("use_topdown_camera_control", &guik::LightViewer::use_topdown_camera_control, py::arg("distance") = 80.0, py::arg("theta") = 0.0)
    .def("use_fps_camera_control", &guik::LightViewer::use_fps_camera_control, "", py::arg("fovy_deg") = 60.0)

    .def("pick_info", &guik::LightViewer::pick_info, py::arg("p"), py::arg("window") = 2)
    .def("pick_depth", &guik::LightViewer::pick_depth, py::arg("p"), py::arg("window") = 2)
    .def("unproject", &guik::LightViewer::unproject, py::arg("p"), py::arg("depth"));

  // AsyncLightViewerContext
  py::class_<guik::AsyncLightViewerContext, std::shared_ptr<guik::AsyncLightViewerContext>>(guik_, "AsyncLightViewerContext")
    .def("clear", &guik::AsyncLightViewerContext::clear)
    .def("clear_text", &guik::AsyncLightViewerContext::clear_text)
    .def("append_text", &guik::AsyncLightViewerContext::append_text)
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
    .def("reset_center", &guik::AsyncLightViewerContext::reset_center)
    .def("lookat", &guik::AsyncLightViewerContext::lookat)
    .def("set_draw_xy_grid", &guik::AsyncLightViewerContext::set_draw_xy_grid, "")
    .def(
      "use_orbit_camera_control",
      &guik::AsyncLightViewerContext::use_orbit_camera_control,
      "",
      py::arg("distance") = 80.0,
      py::arg("theta") = 0.0,
      py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_orbit_camera_control_xz", &guik::AsyncLightViewerContext::use_orbit_camera_control_xz, "", py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = 0.0)
    .def("use_topdown_camera_control", &guik::AsyncLightViewerContext::use_topdown_camera_control, "", py::arg("distance") = 80.0, py::arg("theta") = 0.0)
    .def("use_fps_camera_control", &guik::AsyncLightViewerContext::use_fps_camera_control, py::arg("fovy_deg") = 60.0);

  // AsyncLightViewer
  py::class_<guik::AsyncLightViewer, std::unique_ptr<guik::AsyncLightViewer, py::nodelete>>(guik_, "AsyncLightViewer")
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false,
      py::arg("title") = "screen")
    .def(
      "async_sub_viewer",
      [](guik::AsyncLightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.async_sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      })

    // LightViewerContext methods
    .def("clear", &guik::AsyncLightViewer::clear)
    .def("clear_text", &guik::AsyncLightViewer::clear_text)
    .def("append_text", &guik::AsyncLightViewer::append_text, py::arg("text"))
    .def("register_ui_callback", &guik::AsyncLightViewer::register_ui_callback, py::arg("callback_name"), py::arg("callback"))

    .def(
      "remove_drawable",
      [](guik::AsyncLightViewer& context, const std::string& pattern, bool regex) {
        if (regex) {
          context.remove_drawable(std::regex(pattern));
        } else {
          context.remove_drawable(pattern);
        }
      },
      py::arg("pattern"),
      py::arg("regex") = false)
    .def("reset_center", &guik::AsyncLightViewer::reset_center)
    .def("lookat", &guik::AsyncLightViewer::lookat, py::arg("pt"))
    .def("set_draw_xy_grid", &guik::AsyncLightViewer::set_draw_xy_grid, py::arg("enable"))
    .def("use_orbit_camera_control", &guik::AsyncLightViewer::use_orbit_camera_control, py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = -60.0 * M_PI / 180.0)
    .def("use_orbit_camera_control_xz", &guik::AsyncLightViewer::use_orbit_camera_control_xz, py::arg("distance") = 80.0, py::arg("theta") = 0.0, py::arg("phi") = 0.0)
    .def("use_topdown_camera_control", &guik::AsyncLightViewer::use_topdown_camera_control, py::arg("distance") = 80.0, py::arg("theta") = 0.0);

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
  guik_.def("anon", &guik::anon, "");
  guik_.def(
    "viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return instance(size, background, title); },
    "",
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen");
  guik_.def("destroy", &guik::destroy, "");
  guik_.def(
    "async_viewer",
    [](const Eigen::Vector2i& size, bool background, const std::string& title) { return async_instance(size, background, title); },
    "",
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false,
    py::arg("title") = "screen");
  guik_.def("async_destroy", &guik::async_destroy, "");
  guik_.def("async_wait", &guik::async_wait, "");
}
