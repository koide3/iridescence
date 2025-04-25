#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <iostream>
#include <imgui.h>
#include <imgui_internal.h>
#include <Eigen/Core>

#include <glk/texture.hpp>

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
  imgui_.attr("Cond_FirstUseEver") = py::int_(static_cast<int>(ImGuiCond_FirstUseEver));
  imgui_.attr("Cond_Appearing") = py::int_(static_cast<int>(ImGuiCond_Appearing));

  imgui_.attr("Dir_Left") = py::int_(static_cast<int>(ImGuiDir_Left));
  imgui_.attr("Dir_Right") = py::int_(static_cast<int>(ImGuiDir_Right));
  imgui_.attr("Dir_Up") = py::int_(static_cast<int>(ImGuiDir_Up));
  imgui_.attr("Dir_Down") = py::int_(static_cast<int>(ImGuiDir_Down));
  imgui_.attr("Dir_None") = py::int_(static_cast<int>(ImGuiDir_None));

  imgui_.attr("ButtonFlags_None") = py::int_(static_cast<int>(ImGuiButtonFlags_None));
  imgui_.attr("ButtonFlags_MouseButtonLeft") = py::int_(static_cast<int>(ImGuiButtonFlags_MouseButtonLeft));
  imgui_.attr("ButtonFlags_MouseButtonRight") = py::int_(static_cast<int>(ImGuiButtonFlags_MouseButtonRight));
  imgui_.attr("ButtonFlags_MouseButtonMiddle") = py::int_(static_cast<int>(ImGuiButtonFlags_MouseButtonMiddle));

  imgui_.attr("MouseButton_Right") = py::int_(static_cast<int>(ImGuiMouseButton_Right));
  imgui_.attr("MouseButton_Middle") = py::int_(static_cast<int>(ImGuiMouseButton_Middle));
  imgui_.attr("MouseButton_Left") = py::int_(static_cast<int>(ImGuiMouseButton_Left));

  imgui_.attr("DockNodeFlags_None") = py::int_(static_cast<int>(ImGuiDockNodeFlags_None));
  imgui_.attr("DockNodeFlags_KeepAliveOnly") = py::int_(static_cast<int>(ImGuiDockNodeFlags_KeepAliveOnly));
  imgui_.attr("DockNodeFlags_NoDockingInCentralNode") = py::int_(static_cast<int>(ImGuiDockNodeFlags_NoDockingInCentralNode));
  imgui_.attr("DockNodeFlags_PassthruCentralNode") = py::int_(static_cast<int>(ImGuiDockNodeFlags_PassthruCentralNode));
  imgui_.attr("DockNodeFlags_NoSplit") = py::int_(static_cast<int>(ImGuiDockNodeFlags_NoSplit));
  imgui_.attr("DockNodeFlags_NoResize") = py::int_(static_cast<int>(ImGuiDockNodeFlags_NoResize));
  imgui_.attr("DockNodeFlags_AutoHideTabBar") = py::int_(static_cast<int>(ImGuiDockNodeFlags_AutoHideTabBar));

  // macros
  imgui_.def("IM_COL32", [](int r, int g, int b, int a) { return IM_COL32(r, g, b, a); }, py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"));

  // structs
  py::class_<ImVec2>(imgui_, "ImVec2", py::buffer_protocol())
    .def_buffer([](ImVec2& m) -> py::buffer_info { return py::buffer_info(&m.x, sizeof(float), py::format_descriptor<float>::format(), 1, {2}, {sizeof(float)}); })
    .def(py::init([](float x, float y) { return std::make_unique<ImVec2>(x, y); }))
    .def(py::init([](py::array_t<float> x) {
      auto r = x.unchecked<1>();
      return std::make_unique<ImVec2>(r(0), r(1));
    }));

  // functions
  imgui_.def(
    "begin",
    [](const std::string& name, bool open, int flags) { return std::make_tuple(ImGui::Begin(name.c_str(), &open, flags), open); },
    py::arg("name"),
    py::arg("open") = true,
    py::arg("flags") = 0);
  imgui_.def("end", [] { ImGui::End(); });
  imgui_.def("get_id", [](const std::string& name) { return ImGui::GetID(name.c_str()); }, py::arg("name"));

  imgui_.def("open_popup", [](const std::string& name) { ImGui::OpenPopup(name.c_str()); }, py::arg("name"));
  imgui_.def("begin_popup", &ImGui::BeginPopup, py::arg("id"), py::arg("flags"));
  imgui_.def(
    "begin_popup_modal",
    [](const std::string& name, bool open, int flags) { return std::make_tuple(ImGui::BeginPopupModal(name.c_str(), &open, flags), open); },
    py::arg("name"),
    py::arg("open") = true,
    py::arg("flags") = 0);
  imgui_.def("end_popup", &ImGui::EndPopup);
  imgui_.def("close_current_popup", &ImGui::CloseCurrentPopup);

  imgui_.def(
    "set_next_window_pos",
    [](const Eigen::Vector2i& pos, int cond, const Eigen::Vector2i& pivot) { ImGui::SetNextWindowPos(ImVec2(pos[0], pos[1]), cond, ImVec2(pivot[0], pivot[1])); },
    "",
    py::arg("pos"),
    py::arg("cond") = 0,
    py::arg("pivot") = Eigen::Vector2i(0, 0));
  imgui_
    .def("set_next_window_size", [](const Eigen::Vector2i& size, int cond) { ImGui::SetNextWindowSize(ImVec2(size[0], size[1]), cond); }, "", py::arg("size"), py::arg("cond") = 0);

  imgui_.def("separator", &ImGui::Separator);
  imgui_.def("same_line", &ImGui::SameLine, "", py::arg("offset_from_start_x") = 0.0, py::arg("spacing") = -1.0);
  imgui_.def("newline", &ImGui::NewLine);
  imgui_.def("spacing", &ImGui::Spacing);

  imgui_.def("text", [](const std::string& text) { ImGui::Text("%s", text.c_str()); }, py::arg("text"));
  imgui_.def(
    "input_text",
    [](const std::string& label, const std::string& text, int flags, int buffer_size) {
      std::vector<char> buffer(buffer_size, 0);
      std::copy(text.begin(), text.end(), buffer.begin());
      const bool changed = ImGui::InputText(label.c_str(), buffer.data(), buffer_size, flags);
      return std::make_pair(changed, std::string(buffer.data()));
    },
    "",
    py::arg("label"),
    py::arg("text"),
    py::arg("flags") = 0,
    py::arg("buffer_size") = 256);
  imgui_.def("button", [](const std::string& label) { return ImGui::Button(label.c_str()); }, py::arg("label"));
  imgui_.def("arrow_button", ImGui::ArrowButton, py::arg("id"), py::arg("dir"));

  imgui_.def(
    "invisible_button",
    [](const std::string& id, const Eigen::Vector2i& size, int flags) { return ImGui::InvisibleButton(id.c_str(), ImVec2(size[0], size[1]), flags); },
    "",
    py::arg("id"),
    py::arg("size"),
    py::arg("flags") = 0);

  imgui_.def("checkbox", [](const std::string& label, bool v) { return std::make_tuple(ImGui::Checkbox(label.c_str(), &v), v); }, py::arg("label"), py::arg("v"));
  imgui_.def(
    "drag_int",
    [](const std::string& label, int v, int v_speed, int v_min, int v_max, const std::string& format) {
      return std::make_tuple(ImGui::DragInt(label.c_str(), &v, v_speed, v_min, v_max, format.c_str()), v);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1,
    py::arg("v_min") = 0,
    py::arg("v_max") = 0,
    py::arg("format") = "%d");
  imgui_.def(
    "drag_float",
    [](const std::string& label, float v, float v_speed, float v_min, float v_max, const std::string& format, float power) {
      return std::make_tuple(ImGui::DragFloat(label.c_str(), &v, v_speed, v_min, v_max, format.c_str(), power), v);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1.0f,
    py::arg("v_min") = 0.0f,
    py::arg("v_max") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);

  imgui_.def("push_button_repeat", &ImGui::PushButtonRepeat, py::arg("repeat"));
  imgui_.def("pop_button_repeat", &ImGui::PopButtonRepeat);

  imgui_.def(
    "image",
    [](
      const std::shared_ptr<glk::Texture>& texture,
      const Eigen::Vector2i& size,
      const Eigen::Vector2f& uv0,
      const Eigen::Vector2f& uv1,
      const Eigen::Vector4f& tint_col,
      const Eigen::Vector4f& border_col) {
      ImGui::Image(
        reinterpret_cast<void*>(texture->id()),
        ImVec2(size[0], size[1]),
        ImVec2(uv0[0], uv0[1]),
        ImVec2(uv1[0], uv1[1]),
        ImVec4(tint_col[0], tint_col[1], tint_col[2], tint_col[3]),
        ImVec4(border_col[0], border_col[1], border_col[2], border_col[3]));
    },
    "",
    py::arg("texture"),
    py::arg("size"),
    py::arg("uv0") = Eigen::Vector2f(0.0, 0.0),
    py::arg("uv1") = Eigen::Vector2f(1.0, 1.0),
    py::arg("tint_col") = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f),
    py::arg("border_col") = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f));

  // Tab bars
  imgui_.def("begin_tab_bar", &ImGui::BeginTabBar, py::arg("id"), py::arg("flags") = 0);
  imgui_.def("end_tab_bar", &ImGui::EndTabBar);
  imgui_.def("begin_tab_item", [](const char* label) { return ImGui::BeginTabItem(label); }, py::arg("label"));
  imgui_.def("end_tab_item", &ImGui::EndTabItem);
  imgui_.def("tab_item_button", &ImGui::TabItemButton, py::arg("label"), py::arg("flags") = 0);

  imgui_.def("show_demo_window", [] { ImGui::ShowDemoWindow(); });

  // Docking
  imgui_.def(
    "dockspace",
    [](unsigned int id, const Eigen::Vector2i& size, int flags) { return ImGui::DockSpace(id, ImVec2(size[0], size[1]), flags); },
    py::arg("id"),
    py::arg("size") = Eigen::Vector2i(0, 0),
    py::arg("flags") = 0);
  imgui_.def("dockspace_over_viewport", [](int flags) { return ImGui::DockSpaceOverViewport(nullptr, flags); }, py::arg("flags") = 0);
  imgui_.def("set_next_window_dock_id", &ImGui::SetNextWindowDockID, py::arg("dock_id"), py::arg("cond") = 0);

  // Dock builder
  imgui_.def("dock_builder_dock_window", &ImGui::DockBuilderDockWindow, py::arg("window_name"), py::arg("node_id"));
  imgui_.def("dock_builder_add_node", &ImGui::DockBuilderAddNode, py::arg("node_id") = 0, py::arg("flags") = 0);
  imgui_.def("dock_builder_remove_node", &ImGui::DockBuilderRemoveNode, py::arg("node_id"));
  imgui_.def(
    "dock_builder_split_node",
    [](unsigned int node_id, int split_dir, float size_ratio_for_node_at_dir) -> std::pair<unsigned int, unsigned int> {
      ImGuiID id_at_dir, id_at_opposite_dir;
      ImGui::DockBuilderSplitNode(node_id, split_dir, size_ratio_for_node_at_dir, &id_at_dir, &id_at_opposite_dir);
      return std::make_pair(id_at_dir, id_at_opposite_dir);
    },
    py::arg("node_id"),
    py::arg("split_dir"),
    py::arg("size_ratio_for_node_at_dir"));
  imgui_.def("dock_builder_finish", [](unsigned int node_id) { ImGui::DockBuilderFinish(node_id); }, py::arg("node_id"));

  // DrawList
  py::class_<ImDrawList, std::shared_ptr<ImDrawList>>(imgui_, "ImDrawList")
    .def(
      "add_line",
      [](ImDrawList* draw_list, const Eigen::Vector2i& p0, const Eigen::Vector2i& p1, unsigned int color, float thickness) {
        draw_list->AddLine(ImVec2(p0[0], p0[1]), ImVec2(p1[0], p1[1]), color, thickness);
      },
      py::arg("p0"),
      py::arg("p1"),
      py::arg("col"),
      py::arg("thickness") = 1.0f)
    .def(
      "add_rect_filled",
      [](ImDrawList* draw_list, const Eigen::Vector2i& p0, const Eigen::Vector2i& p1, unsigned int color) {
        draw_list->AddRectFilled(ImVec2(p0[0], p0[1]), ImVec2(p1[0], p1[1]), color);
      },
      py::arg("p0"),
      py::arg("p1"),
      py::arg("color"))
    .def(
      "add_rect",
      [](ImDrawList* draw_list, const Eigen::Vector2i& p0, const Eigen::Vector2i& p1, unsigned int color) {
        draw_list->AddRect(ImVec2(p0[0], p0[1]), ImVec2(p1[0], p1[1]), color);
      },
      py::arg("p0"),
      py::arg("p1"),
      py::arg("color"))
    .def(
      "add_circle",
      [](ImDrawList* draw_list, const Eigen::Vector2i& center, float radius, unsigned int color, int num_segments, float thickness) {
        draw_list->AddCircle(ImVec2(center[0], center[1]), radius, color, num_segments, thickness);
      },
      py::arg("center"),
      py::arg("radius"),
      py::arg("color"),
      py::arg("num_segments") = 0,
      py::arg("thickness") = 1.0f)
    .def(
      "add_circle_filled",
      [](ImDrawList* draw_list, const Eigen::Vector2i& center, float radius, unsigned int color, int num_segments) {
        draw_list->AddCircleFilled(ImVec2(center[0], center[1]), radius, color, num_segments);
      },
      py::arg("center"),
      py::arg("radius"),
      py::arg("color"),
      py::arg("num_segments") = 0)
    .def(
      "add_text",
      [](ImDrawList* draw_list, const Eigen::Vector2i& pos, unsigned int color, const std::string& text) { draw_list->AddText(ImVec2(pos[0], pos[1]), color, text.c_str()); },
      py::arg("pos"),
      py::arg("color"),
      py::arg("text"))
    //
    ;

  imgui_.def("get_window_draw_list", [] {
    auto draw_list = ImGui::GetWindowDrawList();
    return std::shared_ptr<ImDrawList>(draw_list, [](ImDrawList* ptr) {});
  });

  // IO
  py::class_<ImGuiIO>(imgui_, "IO")
    .def_readonly("want_capture_keyboard", &ImGuiIO::WantCaptureKeyboard)
    .def_readonly("want_capture_mouse", &ImGuiIO::WantCaptureMouse)
    .def_readonly("framerate", &ImGuiIO::Framerate)
    .def_readonly("delta_time", &ImGuiIO::DeltaTime)

    .def_property_readonly("mouse_pos", [](const ImGuiIO& io) { return py::array(2, &io.MousePos.x); })
    .def_property_readonly("mouse_down", [](const ImGuiIO& io) { return py::array(5, io.MouseDown); })
    .def_property_readonly("mouse_clicked", [](const ImGuiIO& io) { return py::array(5, io.MouseClicked); })

    .def_readonly("mouse_wheel", &ImGuiIO::MouseWheel)
    .def_readonly("mouse_wheel_h", &ImGuiIO::MouseWheelH)
    .def_readonly("key_ctrl", &ImGuiIO::KeyCtrl)
    .def_readonly("key_shift", &ImGuiIO::KeyShift)
    .def_readonly("key_alt", &ImGuiIO::KeyAlt)
    .def_readonly("key_super", &ImGuiIO::KeySuper)
    .def_property_readonly("keys_down", [](const ImGuiIO& io) { return py::array(512, io.KeysDown); })
    .def_property_readonly("nav_inputs", [](const ImGuiIO& io) { return py::array(ImGuiNavInput_COUNT, io.NavInputs); });

  imgui_.def("get_io", [] { return ImGui::GetIO(); });
  imgui_.def("is_mouse_clicked", [](int button, bool repeat) { return ImGui::IsMouseClicked(button, repeat); }, "", py::arg("button") = 0, py::arg("repeat") = false);

  imgui_.def("get_mouse_pos", [] {
    auto pos = ImGui::GetMousePos();
    return Eigen::Vector2f(pos[0], pos[1]);
  });

  imgui_.def("get_window_pos", [] {
    auto pos = ImGui::GetWindowPos();
    return Eigen::Vector2f(pos[0], pos[1]);
  });

  imgui_.def("get_cursor_screen_pos", [] {
    auto pos = ImGui::GetCursorScreenPos();
    return Eigen::Vector2f(pos[0], pos[1]);
  });
  imgui_.def("get_content_region_avail", [] {
    auto pos = ImGui::GetContentRegionAvail();
    return Eigen::Vector2f(pos[0], pos[1]);
  });

  imgui_.def("is_item_hovered", &ImGui::IsItemHovered, py::arg("flags") = 0);
  imgui_.def("is_item_active", &ImGui::IsItemActive);
  imgui_.def("is_item_focused", &ImGui::IsItemFocused);
}