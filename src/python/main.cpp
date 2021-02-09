#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <portable-file-dialogs.h>

namespace py = pybind11;


PYBIND11_MODULE(pyridescence, m) {
  // modules
  py::module_ glk_ = m.def_submodule("glk", "");
  py::module_ guik_ = m.def_submodule("guik", "");
  py::module_ primitives_ = glk_.def_submodule("primitives", "");

  // glk guik classess
  py::class_<glk::Drawable, std::shared_ptr<glk::Drawable>>(glk_, "Drawable");
  py::class_<guik::ShaderSetting, std::shared_ptr<guik::ShaderSetting>>(guik_, "ShaderSetting")
    .def("add", &guik::ShaderSetting::add<float>)
    .def("make_transparent", &guik::ShaderSetting::make_transparent);

  // glk methods
  glk_.def("create_pointcloud_buffer", [](const Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& x) -> glk::Drawable::Ptr { return std::make_shared<glk::PointCloudBuffer>(x.data(), sizeof(float) * 3, x.rows()); } );

  // primitives
  primitives_.def("sphere", [] { guik::LightViewer::instance(); return glk::Primitives::sphere(); });
  primitives_.def("cube", [] { guik::LightViewer::instance(); return glk::Primitives::cube(); });
  primitives_.def("coordinate_system", [] { guik::LightViewer::instance(); return glk::Primitives::coordinate_system(); });

  // guik methods
  guik_.def("Rainbow", []() -> guik::ShaderSetting::Ptr { return std::make_shared<guik::Rainbow>(); });
  guik_.def("VertexColor", []() -> guik::ShaderSetting::Ptr { return std::make_shared<guik::VertexColor>(); });
  guik_.def("FlatColor", [](float r, float g, float b, float a) -> guik::ShaderSetting::Ptr { return std::make_shared<guik::FlatColor>(r, g, b, a); });

  guik_.def("Rainbow", [](const Eigen::Matrix4f& transform) -> guik::ShaderSetting::Ptr { return std::make_shared<guik::Rainbow>(transform); });
  guik_.def("VertexColor", [](const Eigen::Matrix4f& transform) -> guik::ShaderSetting::Ptr { return std::make_shared<guik::VertexColor>(transform); });
  guik_.def("FlatColor", [](float r, float g, float b, float a, const Eigen::Matrix4f& transform) -> guik::ShaderSetting::Ptr { return std::make_shared<guik::FlatColor>(r, g, b, a, transform); });

  // LightViewerContext
  py::class_<guik::LightViewerContext, std::shared_ptr<guik::LightViewerContext>>(guik_, "LightViewerContext")
    .def("clear", &guik::LightViewerContext::clear)
    .def("clear_text", &guik::LightViewerContext::clear_text)
    .def("append_text", &guik::LightViewerContext::append_text)
    .def("remove_drawable", &guik::LightViewerContext::remove_drawable)
    .def("update_drawable", &guik::LightViewerContext::update_drawable)
    .def("reset_center", &guik::LightViewerContext::reset_center)
    .def("lookat", &guik::LightViewerContext::lookat)
    .def("use_orbit_camera_control", &guik::LightViewerContext::use_orbit_camera_control)
    .def("use_orbit_camera_control_xz", &guik::LightViewerContext::use_orbit_camera_control_xz)
    .def("use_topdown_camera_control", &guik::LightViewerContext::use_topdown_camera_control)
    ;

  // LightViewer
  py::class_<guik::LightViewer, guik::LightViewerContext, std::shared_ptr<guik::LightViewer>>(guik_, "LightViewer")
    .def_static("instance", [] { return guik::LightViewer::instance(); })
    .def("sub_viewer", [] (guik::LightViewer& viewer, const std::string& name) { return viewer.sub_viewer(name); })
    .def("sub_viewer", [] (guik::LightViewer& viewer, const std::string& name, const std::tuple<int, int>& size) { return viewer.sub_viewer(name, Eigen::Vector2i(std::get<0>(size), std::get<1>(size))); })

    .def("spin", &guik::LightViewer::spin, "")
    .def("spin_once", &guik::LightViewer::spin_once, "")
    .def("spin_until_click", &guik::LightViewer::spin_until_click, "")

    // LightViewerContext methods
    .def("clear", &guik::LightViewer::clear)
    .def("clear_text", &guik::LightViewer::clear_text)
    .def("append_text", &guik::LightViewer::append_text)
    .def("register_ui_callback", &guik::LightViewer::register_ui_callback)

    .def("remove_drawable", &guik::LightViewer::remove_drawable)
    .def("update_drawable", &guik::LightViewer::update_drawable)
    .def("reset_center", &guik::LightViewer::reset_center)
    .def("lookat", &guik::LightViewer::lookat)
    .def("use_orbit_camera_control", &guik::LightViewer::use_orbit_camera_control)
    .def("use_orbit_camera_control_xz", &guik::LightViewer::use_orbit_camera_control_xz)
    .def("use_topdown_camera_control", &guik::LightViewer::use_topdown_camera_control)
    ;

  // portable-file-diaglos
  py::module_ pfd_ = m.def_submodule("pfd", "");
  pfd_.def("notify", [] (const std::string& title, const std::string& message) { pfd::notify(title, message); }, "");
  pfd_.def("select_folder", [] (const std::string& title, const std::string& default_path) { return pfd::select_folder(title, default_path).result(); }, "", py::arg("title"), py::arg("default_path") = "");
  pfd_.def("save_file", [] (const std::string& title, const std::string& default_path) { return pfd::save_file(title, default_path).result(); }, "", py::arg("title"), py::arg("default_path") = "");
  pfd_.def("open_file", [] (const std::string& title, const std::string& default_path) { pfd::open_file dialog(title, default_path); return dialog.result().empty() ? "" : dialog.result().front(); }, "", py::arg("title"), py::arg("default_path") = "");

  // imgui
  py::module_ imgui_ = m.def_submodule("imgui", "");
  imgui_.def("begin", [] (const std::string& name) { return ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize); });
  imgui_.def("end", [] { ImGui::End(); });

  imgui_.def("text", [](const std::string& text) { ImGui::Text("%s", text.c_str()); });
  imgui_.def("button", [](const std::string& label) { return ImGui::Button(label.c_str()); });
  imgui_.def("checkbox", [](const std::string& label, bool v) { return std::make_tuple(ImGui::Checkbox(label.c_str(), &v), v); });
  imgui_.def("drag_float", [](const std::string& label, float v, float v_speed, float v_min, float v_max, const std::string& format, float power)
    { return std::make_tuple(ImGui::DragFloat(label.c_str(), &v, v_speed, v_min, v_max, format.c_str(), power), v); }, "",
    py::arg("label"), py::arg("v"), py::arg("v_speed") = 1.0f, py::arg("v_min") = 0.0f, py::arg("v_max") = 0.0f, py::arg("format") = "%.3f", py::arg("power") = 1.0f
  );

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}