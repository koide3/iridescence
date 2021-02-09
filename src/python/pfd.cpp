#include <pybind11/pybind11.h>

#include <portable-file-dialogs.h>

namespace py = pybind11;

void define_pfd(py::module_& m) {
  // portable-file-dialogs
  py::module_ pfd_ = m.def_submodule("pfd", "");
  pfd_.def("notify", [] (const std::string& title, const std::string& message) { pfd::notify(title, message); }, "");
  pfd_.def("select_folder", [] (const std::string& title, const std::string& default_path) { return pfd::select_folder(title, default_path).result(); }, "", py::arg("title"), py::arg("default_path") = "");
  pfd_.def("save_file", [] (const std::string& title, const std::string& default_path) { return pfd::save_file(title, default_path).result(); }, "", py::arg("title"), py::arg("default_path") = "");
  pfd_.def("open_file", [] (const std::string& title, const std::string& default_path) { pfd::open_file dialog(title, default_path); return dialog.result().empty() ? "" : dialog.result().front(); }, "", py::arg("title"), py::arg("default_path") = "");
}