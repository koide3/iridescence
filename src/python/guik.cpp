#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <boost/filesystem.hpp>

#include <glk/path.hpp>
#include <glk/texture.hpp>
#include <guik/recent_files.hpp>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace py = pybind11;

std::shared_ptr<guik::LightViewer> instance(const Eigen::Vector2i& size, bool background) {
  static bool is_first = true;
  if (is_first) {
    py::gil_scoped_acquire acquire;
    py::object pyridescence = py::module::import("pyridescence");
    boost::filesystem::path path(pyridescence.attr("__file__").cast<std::string>());

    glk::set_data_path(path.parent_path().string() + "/data");
    is_first = false;
  }

  static std::shared_ptr<guik::LightViewer> inst = guik::LightViewer::instance(size, background);
  return inst;
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
    .def("set_model_matrix", &guik::ModelControl::set_model_matrix, py::arg("model_matrix"));

  // guik::CameraControl
  py::class_<guik::CameraControl, std::shared_ptr<guik::CameraControl>>(guik_, "CameraControl");

  // guik::ProjectionControl
  py::class_<guik::ProjectionControl, std::shared_ptr<guik::ProjectionControl>>(guik_, "ProjectionControl");

  // LightViewerContext
  py::class_<guik::LightViewerContext, std::shared_ptr<guik::LightViewerContext>>(guik_, "LightViewerContext")
    .def("set_size", &guik::LightViewerContext::set_size)
    .def("set_clear_color", &guik::LightViewerContext::set_clear_color)
    .def("set_pos", &guik::LightViewerContext::set_pos, "", py::arg("pos"), py::arg("cond") = static_cast<int>(ImGuiCond_FirstUseEver))
    .def("clear", &guik::LightViewerContext::clear)
    .def("clear_text", &guik::LightViewerContext::clear_text)
    .def("append_text", &guik::LightViewerContext::append_text)
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
    .def("get_camera_control", &guik::LightViewerContext::get_camera_control, "")
    .def("set_camera_control", &guik::LightViewerContext::set_camera_control, "")
    .def("get_projection_control", &guik::LightViewerContext::get_projection_control, "")
    .def("set_projection_control", &guik::LightViewerContext::set_projection_control, "");

  // LightViewer
  py::class_<guik::LightViewer, guik::LightViewerContext, std::shared_ptr<guik::LightViewer>>(guik_, "LightViewer")
    .def_static(
      "instance",
      [](const Eigen::Vector2i& size, bool background) { return instance(size, background); },
      py::arg("size") = Eigen::Vector2i(1920, 1080),
      py::arg("background") = false)
    .def("sub_viewer", [](guik::LightViewer& viewer, const std::string& name) { return viewer.sub_viewer(name); })
    .def(
      "sub_viewer",
      [](guik::LightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) {
        return viewer.sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size)));
      })

    .def("close", &guik::LightViewer::close)
    .def("spin", &guik::LightViewer::spin)
    .def("spin_once", &guik::LightViewer::spin_once)
    .def("toggle_spin_once", &guik::LightViewer::toggle_spin_once)
    .def("spin_until_click", &guik::LightViewer::spin_until_click)

    .def("enable_vsync", &guik::LightViewer::enable_vsync)
    .def("enable_docking", &guik::LightViewer::enable_docking)

    .def("clear_images", &guik::LightViewer::clear_images)
    .def("remove_image", &guik::LightViewer::remove_image)
    .def("update_image", &guik::LightViewer::update_image, py::arg("name"), py::arg("image"), py::arg("scale") = -1.0)

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

    .def("pick_info", &guik::LightViewer::pick_info, py::arg("p"), py::arg("window") = 2)
    .def("pick_depth", &guik::LightViewer::pick_depth, py::arg("p"), py::arg("window") = 2)
    .def("unproject", &guik::LightViewer::unproject, py::arg("p"), py::arg("depth"));

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
    [](const Eigen::Vector2i& size, bool background) { return instance(size, background); },
    "",
    py::arg("size") = Eigen::Vector2i(1920, 1080),
    py::arg("background") = false);
  guik_.def("destroy", &guik::destroy, "");
}