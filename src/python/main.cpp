#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <glk/path.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <portable-file-dialogs.h>

namespace py = pybind11;

void define_glk(py::module_& m);
void define_guik(py::module_& m);
void define_pfd(py::module_& m);
void define_imgui(py::module_& m);

PYBIND11_MODULE(pyridescence, m) {
  // modules
  define_glk(m);
  define_guik(m);
  define_pfd(m);
  define_imgui(m);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}