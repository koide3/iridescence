#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <imgui.h>
#include <Eigen/Core>

namespace py = pybind11;

void define_imgui(py::module_& m) {
    // imgui
  py::module_ imgui_ = m.def_submodule("imgui", "");

  // enums
  imgui_.attr("WindowFlags_None") = py::int_(static_cast<int>(ImGuiWindowFlags_None));
  imgui_.attr("WindowFlags_NoTitleBar") = py::int_(static_cast<int>(ImGuiWindowFlags_NoTitleBar));
  imgui_.attr("WindowFlags_NoResize") = py::int_(static_cast<int>(ImGuiWindowFlags_NoResize));
  imgui_.attr("WindowFlags_NoMove") = py::int_(static_cast<int>(ImGuiWindowFlags_NoMove));
  imgui_.attr("WindowFlags_NoScrollbar") = py::int_(static_cast<int>(ImGuiWindowFlags_NoScrollbar));
  imgui_.attr("WindowFlags_AlwaysAutoResize") = py::int_(static_cast<int>(ImGuiWindowFlags_AlwaysAutoResize));
  imgui_.attr("WindowFlags_NoBackground") = py::int_(static_cast<int>(ImGuiWindowFlags_NoBackground));

  imgui_.attr("Cond_Always") = py::int_(static_cast<int>(ImGuiCond_Always));
  imgui_.attr("Cond_Once") = py::int_(static_cast<int>(ImGuiCond_Once));
  imgui_.attr("Cond_FirstUserEver") = py::int_(static_cast<int>(ImGuiCond_FirstUseEver));
  imgui_.attr("Cond_Appearing") = py::int_(static_cast<int>(ImGuiCond_Appearing));

  imgui_.attr("Dir_Left") = py::int_(static_cast<int>(ImGuiDir_Left));
  imgui_.attr("Dir_Right") = py::int_(static_cast<int>(ImGuiDir_Right));
  imgui_.attr("Dir_Up") = py::int_(static_cast<int>(ImGuiDir_Up));
  imgui_.attr("Dir_Down") = py::int_(static_cast<int>(ImGuiDir_Down));
  imgui_.attr("Dir_None") = py::int_(static_cast<int>(ImGuiDir_None));

  // functions
  imgui_.def("begin", [] (const std::string& name, bool open, int flags) { return std::make_tuple(ImGui::Begin(name.c_str(), &open, flags), open); });
  imgui_.def("end", [] { ImGui::End(); });

  imgui_.def("set_next_window_pos", [] (const Eigen::Vector2i& pos, int cond, const Eigen::Vector2i& pivot) { ImGui::SetNextWindowPos(ImVec2(pos[0], pos[1]), cond, ImVec2(pivot[0], pivot[1])); }, "",
    py::arg("pos"), py::arg("cond") = 0, py::arg("pivot") = Eigen::Vector2i(0, 0));
  imgui_.def("set_next_window_size", [] (const Eigen::Vector2i& size, int cond) { ImGui::SetNextWindowSize(ImVec2(size[0], size[1]), cond); }, "",
    py::arg("size"), py::arg("cond") = 0);

  imgui_.def("same_line", &ImGui::SameLine, "", py::arg("offset_from_start_x") = 0.0, py::arg("spacing") = -1.0);
  imgui_.def("separator", &ImGui::Separator);
  imgui_.def("text", [](const std::string& text) { ImGui::Text("%s", text.c_str()); });
  imgui_.def("input_text",
    [](const std::string& label, const std::string& text, int flags, int buffer_size) { 
      std::vector<char> buffer(buffer_size, 0);
      std::copy(text.begin(), text.end(), buffer.begin());
      return std::make_pair(ImGui::InputText(label.c_str(), buffer.data(), buffer_size, flags), std::string(buffer.data()));
    }, "", py::arg("label"), py::arg("text"), py::arg("flags") = 0, py::arg("buffer_size") = 256
  );
  imgui_.def("button", [](const std::string& label) { return ImGui::Button(label.c_str()); });
  imgui_.def("arrow_button", ImGui::ArrowButton);

  imgui_.def("checkbox", [](const std::string& label, bool v) { return std::make_tuple(ImGui::Checkbox(label.c_str(), &v), v); });
  imgui_.def("drag_int", [](const std::string& label, int v, int v_speed, int v_min, int v_max, const std::string& format)
    { return std::make_tuple(ImGui::DragInt(label.c_str(), &v, v_speed, v_min, v_max, format.c_str()), v); }, "",
    py::arg("label"), py::arg("v"), py::arg("v_speed") = 1, py::arg("v_min") = 0, py::arg("v_max") = 0, py::arg("format") = "%d"
  );
  imgui_.def("drag_float", [](const std::string& label, float v, float v_speed, float v_min, float v_max, const std::string& format, float power)
    { return std::make_tuple(ImGui::DragFloat(label.c_str(), &v, v_speed, v_min, v_max, format.c_str(), power), v); }, "",
    py::arg("label"), py::arg("v"), py::arg("v_speed") = 1.0f, py::arg("v_min") = 0.0f, py::arg("v_max") = 0.0f, py::arg("format") = "%.3f", py::arg("power") = 1.0f
  );

  imgui_.def("push_button_repeat", &ImGui::PushButtonRepeat);
  imgui_.def("pop_button_repeat", &ImGui::PopButtonRepeat);

  // IO
  py::class_<ImGuiIO>(imgui_, "IO")
    .def_readonly("want_capture_keyboard", &ImGuiIO::WantCaptureKeyboard)
    .def_readonly("want_capture_mouse", &ImGuiIO::WantCaptureMouse)
    .def_readonly("framerate", &ImGuiIO::Framerate)
    .def_readonly("delta_time", &ImGuiIO::DeltaTime)


    .def_property_readonly("mouse_pos", [] (const ImGuiIO& io) {
      return py::array(2, &io.MousePos.x);
    })
    .def_property_readonly("mouse_down", [] (const ImGuiIO& io) {
      return py::array(5, io.MouseDown);
    })

    .def_readonly("mouse_wheel", &ImGuiIO::MouseWheel)
    .def_readonly("mouse_wheel_h", &ImGuiIO::MouseWheelH)
    .def_readonly("key_ctrl", &ImGuiIO::KeyCtrl)
    .def_readonly("key_shift", &ImGuiIO::KeyShift)
    .def_readonly("key_alt", &ImGuiIO::KeyAlt)
    .def_readonly("key_super", &ImGuiIO::KeySuper)
    .def_property_readonly("keys_down", [] (const ImGuiIO& io) {
      return py::array(512, io.KeysDown);
    })
    .def_property_readonly("nav_inputs", [] (const ImGuiIO& io) {
      return py::array(ImGuiNavInput_COUNT, io.NavInputs);
    })
  ;

  imgui_.def("get_io", [] { return ImGui::GetIO(); });
  imgui_.def("is_mouse_clicked", &ImGui::IsMouseClicked, "", py::arg("button") = 0, py::arg("repeat") = false);
  imgui_.def("get_mouse_pos", [] { auto pos = ImGui::GetMousePos(); return Eigen::Vector2f(pos[0], pos[1]); });
}