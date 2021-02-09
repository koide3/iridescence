#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <glk/path.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>

namespace py = pybind11;


void define_glk(py::module_& m) {
  py::module_ glk_ = m.def_submodule("glk", "");
  py::module_ primitives_ = glk_.def_submodule("primitives", "");

  // classes
  py::class_<glk::Drawable, std::shared_ptr<glk::Drawable>>(glk_, "Drawable");
  py::class_<glk::ScreenEffect, std::shared_ptr<glk::ScreenEffect>>(glk_, "ScreenEffect");
  py::class_<glk::NaiveScreenSpaceAmbientOcclusion, glk::ScreenEffect, std::shared_ptr<glk::NaiveScreenSpaceAmbientOcclusion>>(glk_, "NaiveScreenSpaceAmbientOcclusion")
    .def(py::init<>());
  py::class_<glk::ScreenSpaceAmbientOcclusion, glk::ScreenEffect, std::shared_ptr<glk::ScreenSpaceAmbientOcclusion>>(glk_, "ScreenSpaceAmbientOcclusion")
    .def(py::init<>());
  py::class_<glk::ScreenSpaceLighting, glk::ScreenEffect, std::shared_ptr<glk::ScreenSpaceLighting>> ssli(glk_, "ScreenSpaceLighting");
  ssli.def(py::init<>())
    .def("num_lights", &glk::ScreenSpaceLighting::num_lights, "")
    .def("set_diffuse_model", &glk::ScreenSpaceLighting::set_diffuse_model)
    .def("set_specular_model", &glk::ScreenSpaceLighting::set_specular_model)
    .def("set_occlusion_model", &glk::ScreenSpaceLighting::set_occlusion_model)
    .def("set_iridescence_model", &glk::ScreenSpaceLighting::set_iridescence_model)
    .def("set_light", [](glk::ScreenSpaceLighting& effect, int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color) { return effect.set_light(i, pos, color); }, "")
    .def("set_albedo", &glk::ScreenSpaceLighting::set_albedo, "")
    .def("set_roughness", &glk::ScreenSpaceLighting::set_roughness, "");

  // enums
  py::enum_<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(ssli, "DIFFUSE_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::DIFFUSE_MODEL::ZERO)
    .value("LAMBERT", glk::ScreenSpaceLighting::DIFFUSE_MODEL::LAMBERT)
    .value("DISNEY", glk::ScreenSpaceLighting::DIFFUSE_MODEL::DISNEY)
    .value("NORMALIZED_DISNEY", glk::ScreenSpaceLighting::DIFFUSE_MODEL::NORMALIZED_DISNEY)
    .value("OREN_NAYAR", glk::ScreenSpaceLighting::DIFFUSE_MODEL::OREN_NAYAR)
    .export_values();  

  py::enum_<glk::ScreenSpaceLighting::SPECULAR_MODEL>(ssli, "SPECULAR_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::SPECULAR_MODEL::ZERO)
    .value("PHONG", glk::ScreenSpaceLighting::SPECULAR_MODEL::PHONG)
    .value("BLINN_PHONG", glk::ScreenSpaceLighting::SPECULAR_MODEL::BLINN_PHONG)
    .value("COOK_TORRANCE", glk::ScreenSpaceLighting::SPECULAR_MODEL::COOK_TORRANCE)
    .export_values();  

  py::enum_<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(ssli, "OCCLUSION_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::OCCLUSION_MODEL::ZERO)
    .value("AMBIENT_OCCLUSION", glk::ScreenSpaceLighting::OCCLUSION_MODEL::AMBIENT_OCCLUSION)
    .export_values();  

  py::enum_<glk::ScreenSpaceLighting::IRIDESCENCE_MODEL>(ssli, "IRIDESCENCE_MODEL")
    .value("ZERO", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::ZERO)
    .value("IRIDESCENCE1", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE1)
    .value("IRIDESCENCE2", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE2)
    .value("IRIDESCENCE3", glk::ScreenSpaceLighting::IRIDESCENCE_MODEL::IRIDESCENCE3)
    .export_values();  

 // methods
  glk_.def("set_data_path", &glk::set_data_path, "");
  glk_.def("create_pointcloud_buffer", [](const Eigen::Matrix<float, -1, 3, Eigen::RowMajor>& x) -> glk::Drawable::Ptr { return std::make_shared<glk::PointCloudBuffer>(x.data(), sizeof(float) * 3, x.rows()); } );

  // primitives
  primitives_.def("sphere", [] { return glk::Primitives::sphere(); });
  primitives_.def("cube", [] { return glk::Primitives::cube(); });
  primitives_.def("coordinate_system", [] { return glk::Primitives::coordinate_system(); });

}