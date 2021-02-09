#include <pybind11/pybind11.h>

#include <imgui.h>

namespace py = pybind11;

void define_imgui(py::module_& m) {
    // imgui
  py::module_ imgui_ = m.def_submodule("imgui", "");
  imgui_.attr("WindowFlags_None") = py::int_(static_cast<int>(ImGuiWindowFlags_None));
  imgui_.attr("WindowFlags_NoTitleBar") = py::int_(static_cast<int>(ImGuiWindowFlags_NoTitleBar));
  imgui_.attr("WindowFlags_NoResize") = py::int_(static_cast<int>(ImGuiWindowFlags_NoResize));
  imgui_.attr("WindowFlags_NoMove") = py::int_(static_cast<int>(ImGuiWindowFlags_NoMove));
  imgui_.attr("WindowFlags_NoScrollbar") = py::int_(static_cast<int>(ImGuiWindowFlags_NoScrollbar));
  imgui_.attr("WindowFlags_AlwaysAutoResize") = py::int_(static_cast<int>(ImGuiWindowFlags_AlwaysAutoResize));
  imgui_.attr("WindowFlags_NoBackground") = py::int_(static_cast<int>(ImGuiWindowFlags_NoBackground));

  imgui_.def("begin", [] (const std::string& name, bool open, int flags) { return std::make_tuple(ImGui::Begin(name.c_str(), &open, flags), open); });
  imgui_.def("end", [] { ImGui::End(); });

  imgui_.def("text", [](const std::string& text) { ImGui::Text("%s", text.c_str()); });
  imgui_.def("button", [](const std::string& label) { return ImGui::Button(label.c_str()); });
  imgui_.def("checkbox", [](const std::string& label, bool v) { return std::make_tuple(ImGui::Checkbox(label.c_str(), &v), v); });
  imgui_.def("drag_int", [](const std::string& label, int v, int v_speed, int v_min, int v_max, const std::string& format)
    { return std::make_tuple(ImGui::DragInt(label.c_str(), &v, v_speed, v_min, v_max, format.c_str()), v); }, "",
    py::arg("label"), py::arg("v"), py::arg("v_speed") = 1, py::arg("v_min") = 0, py::arg("v_max") = 0, py::arg("format") = "%d"
  );
  imgui_.def("drag_float", [](const std::string& label, float v, float v_speed, float v_min, float v_max, const std::string& format, float power)
    { return std::make_tuple(ImGui::DragFloat(label.c_str(), &v, v_speed, v_min, v_max, format.c_str(), power), v); }, "",
    py::arg("label"), py::arg("v"), py::arg("v_speed") = 1.0f, py::arg("v_min") = 0.0f, py::arg("v_max") = 0.0f, py::arg("format") = "%.3f", py::arg("power") = 1.0f
  );
}