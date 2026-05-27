#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

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

  imgui_.attr("ColorEditFlags_None") = py::int_(static_cast<int>(ImGuiColorEditFlags_None));
  imgui_.attr("ColorEditFlags_NoAlpha") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoPicker") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoOptions") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoSmallPreview") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoInputs") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoTooltip") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoLabel") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoSidePreview") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoDragDrop") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_NoBorder") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_AlphaBar") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_AlphaPreview") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_AlphaPreviewHalf") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_HDR") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_DisplayRGB") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_DisplayHSV") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_DisplayHex") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_Uint8") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_Float") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_PickerHueBar") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_PickerHueWheel") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_InputRGB") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));
  imgui_.attr("ColorEditFlags_InputHSV") = py::int_(static_cast<int>(ImGuiColorEditFlags_NoAlpha));

  imgui_.attr("TableFlags_None") = py::int_(static_cast<int>(ImGuiTableFlags_None));
  imgui_.attr("TableFlags_Resizable") = py::int_(static_cast<int>(ImGuiTableFlags_Resizable));
  imgui_.attr("TableFlags_Reorderable") = py::int_(static_cast<int>(ImGuiTableFlags_Reorderable));
  imgui_.attr("TableFlags_Hideable") = py::int_(static_cast<int>(ImGuiTableFlags_Hideable));
  imgui_.attr("TableFlags_Sortable") = py::int_(static_cast<int>(ImGuiTableFlags_Sortable));
  imgui_.attr("TableFlags_NoSavedSettings") = py::int_(static_cast<int>(ImGuiTableFlags_NoSavedSettings));
  imgui_.attr("TableFlags_ContextMenuInBody") = py::int_(static_cast<int>(ImGuiTableFlags_ContextMenuInBody));
  imgui_.attr("TableFlags_RowBg") = py::int_(static_cast<int>(ImGuiTableFlags_RowBg));
  imgui_.attr("TableFlags_BordersInnerH") = py::int_(static_cast<int>(ImGuiTableFlags_BordersInnerH));
  imgui_.attr("TableFlags_BordersOuterH") = py::int_(static_cast<int>(ImGuiTableFlags_BordersOuterH));
  imgui_.attr("TableFlags_BordersInnerV") = py::int_(static_cast<int>(ImGuiTableFlags_BordersInnerV));
  imgui_.attr("TableFlags_BordersOuterV") = py::int_(static_cast<int>(ImGuiTableFlags_BordersOuterV));
  imgui_.attr("TableFlags_BordersH") = py::int_(static_cast<int>(ImGuiTableFlags_BordersH));
  imgui_.attr("TableFlags_BordersV") = py::int_(static_cast<int>(ImGuiTableFlags_BordersV));
  imgui_.attr("TableFlags_BordersInner") = py::int_(static_cast<int>(ImGuiTableFlags_BordersInner));
  imgui_.attr("TableFlags_BordersOuter") = py::int_(static_cast<int>(ImGuiTableFlags_BordersOuter));
  imgui_.attr("TableFlags_Borders") = py::int_(static_cast<int>(ImGuiTableFlags_Borders));
  imgui_.attr("TableFlags_NoBordersInBody") = py::int_(static_cast<int>(ImGuiTableFlags_NoBordersInBody));
  imgui_.attr("TableFlags_NoBordersInBodyUntilResize") = py::int_(static_cast<int>(ImGuiTableFlags_NoBordersInBodyUntilResize));
  imgui_.attr("TableFlags_SizingFixedFit") = py::int_(static_cast<int>(ImGuiTableFlags_SizingFixedFit));
  imgui_.attr("TableFlags_SizingFixedSame") = py::int_(static_cast<int>(ImGuiTableFlags_SizingFixedSame));
  imgui_.attr("TableFlags_SizingStretchProp") = py::int_(static_cast<int>(ImGuiTableFlags_SizingStretchProp));
  imgui_.attr("TableFlags_SizingStretchSame") = py::int_(static_cast<int>(ImGuiTableFlags_SizingStretchSame));
  imgui_.attr("TableFlags_NoHostExtendX") = py::int_(static_cast<int>(ImGuiTableFlags_NoHostExtendX));
  imgui_.attr("TableFlags_NoHostExtendY") = py::int_(static_cast<int>(ImGuiTableFlags_NoHostExtendY));
  imgui_.attr("TableFlags_NoKeepColumnsVisible") = py::int_(static_cast<int>(ImGuiTableFlags_NoKeepColumnsVisible));
  imgui_.attr("TableFlags_PreciseWidths") = py::int_(static_cast<int>(ImGuiTableFlags_PreciseWidths));
  imgui_.attr("TableFlags_NoClip") = py::int_(static_cast<int>(ImGuiTableFlags_NoClip));
  imgui_.attr("TableFlags_PadOuterX") = py::int_(static_cast<int>(ImGuiTableFlags_PadOuterX));
  imgui_.attr("TableFlags_NoPadOuterX") = py::int_(static_cast<int>(ImGuiTableFlags_NoPadOuterX));
  imgui_.attr("TableFlags_NoPadInnerX") = py::int_(static_cast<int>(ImGuiTableFlags_NoPadInnerX));
  imgui_.attr("TableFlags_ScrollX") = py::int_(static_cast<int>(ImGuiTableFlags_ScrollX));
  imgui_.attr("TableFlags_ScrollY") = py::int_(static_cast<int>(ImGuiTableFlags_ScrollY));
  imgui_.attr("TableFlags_SortMulti") = py::int_(static_cast<int>(ImGuiTableFlags_SortMulti));
  imgui_.attr("TableFlags_SortTristate") = py::int_(static_cast<int>(ImGuiTableFlags_SortTristate));
  imgui_.attr("TableFlags_SizingMask_") = py::int_(static_cast<int>(ImGuiTableFlags_SizingMask_));

  // macros
  imgui_.def("IM_COL32", [](int r, int g, int b, int a) { return IM_COL32(r, g, b, a); }, py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"));

  // structs
  py::class_<ImVec2>(imgui_, "ImVec2", py::buffer_protocol())
    .def_buffer([](ImVec2& m) -> py::buffer_info { return py::buffer_info(&m.x, sizeof(float), py::format_descriptor<float>::format(), 1, {2}, {sizeof(float)}); })
    .def(py::init([](float x, float y) { return std::make_unique<ImVec2>(x, y); }))
    .def(py::init([](py::array_t<float> x) {
      auto r = x.unchecked<1>();
      return std::make_unique<ImVec2>(r(0), r(1));
    }))
    .def_readwrite("x", &ImVec2::x)
    .def_readwrite("y", &ImVec2::y);

  py::class_<ImVec4>(imgui_, "ImVec4", py::buffer_protocol())
    .def_buffer([](ImVec4& m) -> py::buffer_info { return py::buffer_info(&m.x, sizeof(float), py::format_descriptor<float>::format(), 1, {4}, {sizeof(float)}); })
    .def(py::init([](float x, float y, float z, float w) { return std::make_unique<ImVec4>(x, y, z, w); }))
    .def(py::init([](py::array_t<float> x) {
      auto r = x.unchecked<1>();
      return std::make_unique<ImVec4>(r(0), r(1), r(2), r(3));
    }))
    .def_readwrite("x", &ImVec4::x)
    .def_readwrite("y", &ImVec4::y)
    .def_readwrite("z", &ImVec4::z)
    .def_readwrite("w", &ImVec4::w);

  // IO
  py::class_<ImGuiIO>(imgui_, "IO")
    .def_readonly("want_capture_mouse", &ImGuiIO::WantCaptureMouse)
    .def_readonly("want_capture_keyboard", &ImGuiIO::WantCaptureKeyboard)
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

  py::class_<ImGuiStyle>(imgui_, "Style")
    .def_readwrite("alpha", &ImGuiStyle::Alpha)
    .def_readwrite("disabled_alpha", &ImGuiStyle::DisabledAlpha)
    .def_readwrite("window_padding", &ImGuiStyle::WindowPadding)
    .def_readwrite("window_rounding", &ImGuiStyle::WindowRounding)
    .def_readwrite("window_border_size", &ImGuiStyle::WindowBorderSize)
    .def_readwrite("window_min_size", &ImGuiStyle::WindowMinSize)
    .def_readwrite("window_title_align", &ImGuiStyle::WindowTitleAlign)
    .def_readwrite("window_menu_button_position", &ImGuiStyle::WindowMenuButtonPosition)
    .def_readwrite("child_rounding", &ImGuiStyle::ChildRounding)
    .def_readwrite("child_border_size", &ImGuiStyle::ChildBorderSize)
    .def_readwrite("popup_rounding", &ImGuiStyle::PopupRounding)
    .def_readwrite("popup_border_size", &ImGuiStyle::PopupBorderSize)
    .def_readwrite("frame_padding", &ImGuiStyle::FramePadding)
    .def_readwrite("frame_rounding", &ImGuiStyle::FrameRounding)
    .def_readwrite("frame_border_size", &ImGuiStyle::FrameBorderSize)
    .def_readwrite("item_spacing", &ImGuiStyle::ItemSpacing)
    .def_readwrite("item_inner_spacing", &ImGuiStyle::ItemInnerSpacing)
    .def_readwrite("cell_padding", &ImGuiStyle::CellPadding)
    .def_readwrite("touch_extra_padding", &ImGuiStyle::TouchExtraPadding)
    .def_readwrite("indent_spacing", &ImGuiStyle::IndentSpacing)
    .def_readwrite("columns_min_spacing", &ImGuiStyle::ColumnsMinSpacing)
    .def_readwrite("scrollbar_size", &ImGuiStyle::ScrollbarSize)
    .def_readwrite("scrollbar_rounding", &ImGuiStyle::ScrollbarRounding)
    .def_readwrite("grab_min_size", &ImGuiStyle::GrabMinSize)
    .def_readwrite("grab_rounding", &ImGuiStyle::GrabRounding)
    .def_readwrite("tab_rounding", &ImGuiStyle::TabRounding)
    .def_readwrite("tab_border_size", &ImGuiStyle::TabBorderSize)
    .def_readwrite("tab_min_width_for_close_button", &ImGuiStyle::TabMinWidthForCloseButton)
    .def_readwrite("color_button_position", &ImGuiStyle::ColorButtonPosition)
    .def_readwrite("button_text_align", &ImGuiStyle::ButtonTextAlign)
    .def_readwrite("selectable_text_align", &ImGuiStyle::SelectableTextAlign)
    .def_readwrite("display_window_padding", &ImGuiStyle::DisplayWindowPadding)
    .def_readwrite("display_safe_area_padding", &ImGuiStyle::DisplaySafeAreaPadding)
    .def_readwrite("mouse_cursor_scale", &ImGuiStyle::MouseCursorScale)
    .def_readwrite("anti_aliased_lines", &ImGuiStyle::AntiAliasedLines)
    .def_readwrite("anti_aliased_lines_use_tex", &ImGuiStyle::AntiAliasedLinesUseTex)
    .def_readwrite("anti_aliased_fill", &ImGuiStyle::AntiAliasedFill)
    .def_readwrite("curve_tessellation_tol", &ImGuiStyle::CurveTessellationTol)
    .def_readwrite("circle_tessellation_max_error", &ImGuiStyle::CircleTessellationMaxError)
    .def_property_readonly("colors", [](ImGuiStyle& style) {
      py::list color_list;
      for (int i = 0; i < ImGuiCol_COUNT; ++i) {
        color_list.append(py::array(4, &style.Colors[i].x));
      }
      return color_list;
    });

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
      [](ImDrawList* draw_list, const Eigen::Vector2i& p0, const Eigen::Vector2i& p1, unsigned int color, float rounding, int flags) {
        draw_list->AddRectFilled(ImVec2(p0[0], p0[1]), ImVec2(p1[0], p1[1]), color, rounding, flags);
      },
      py::arg("p0"),
      py::arg("p1"),
      py::arg("color"),
      py::arg("rounding") = 0.0f,
      py::arg("flags") = 0)
    .def(
      "add_rect",
      [](ImDrawList* draw_list, const Eigen::Vector2i& p0, const Eigen::Vector2i& p1, unsigned int color, float rounding, int flags, float thickness) {
        draw_list->AddRect(ImVec2(p0[0], p0[1]), ImVec2(p1[0], p1[1]), color, rounding, flags, thickness);
      },
      py::arg("p0"),
      py::arg("p1"),
      py::arg("color"),
      py::arg("rounding") = 0.0f,
      py::arg("flags") = 0,
      py::arg("thickness") = 1.0f)
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

  // functions
  imgui_.def("get_io", [] { return ImGui::GetIO(); });
  imgui_.def("get_style", [] { return ImGui::GetStyle(); });

  // Demo, Debug, Information
  imgui_.def("show_demo_window", [] { ImGui::ShowDemoWindow(); });
  imgui_.def("show_metrics_window", [] { ImGui::ShowMetricsWindow(); });
  imgui_.def("show_debug_log_window", [] { ImGui::ShowDebugLogWindow(); });
  imgui_.def("show_style_editor", [] { ImGui::ShowStyleEditor(); });
  imgui_.def("show_style_selector", [](const std::string& label) { ImGui::ShowStyleSelector(label.c_str()); }, py::arg("label"));
  imgui_.def("show_font_selector", [](const std::string& label) { ImGui::ShowFontSelector(label.c_str()); }, py::arg("label"));
  imgui_.def("get_version", [] { return ImGui::GetVersion(); });

  // Styles
  imgui_.def("style_colors_dark", &ImGui::StyleColorsDark, py::arg("dst") = nullptr);
  imgui_.def("style_colors_light", &ImGui::StyleColorsLight, py::arg("dst") = nullptr);
  imgui_.def("style_colors_classic", &ImGui::StyleColorsClassic, py::arg("dst") = nullptr);

  // Windows
  imgui_.def(
    "begin",
    [](const std::string& name, bool open, int flags) { return std::make_tuple(ImGui::Begin(name.c_str(), &open, flags), open); },
    py::arg("name"),
    py::arg("open") = true,
    py::arg("flags") = 0);
  imgui_.def("end", [] { ImGui::End(); });

  // Child Windows
  imgui_.def(
    "begin_child",
    [](const std::string& str_id, const Eigen::Vector2i& size, bool border, int flags) { return ImGui::BeginChild(str_id.c_str(), ImVec2(size[0], size[1]), border, flags); },
    py::arg("str_id"),
    py::arg("size") = Eigen::Vector2i(0, 0),
    py::arg("border") = false,
    py::arg("flags") = 0);
  imgui_.def("end_child", &ImGui::EndChild);

  // Windows Utilities
  imgui_.def("is_window_appearing", &ImGui::IsWindowAppearing);
  imgui_.def("is_window_collapsed", &ImGui::IsWindowCollapsed);
  imgui_.def("is_window_focused", &ImGui::IsWindowFocused, py::arg("flags") = 0);
  imgui_.def("is_window_hovered", &ImGui::IsWindowHovered, py::arg("flags") = 0);
  imgui_.def("get_window_draw_list", &ImGui::GetWindowDrawList, py::return_value_policy::reference);
  imgui_.def("get_window_pos", [] {
    ImVec2 pos = ImGui::GetWindowPos();
    return Eigen::Vector2i(static_cast<int>(pos.x), static_cast<int>(pos.y));
  });
  imgui_.def("get_window_size", [] {
    ImVec2 size = ImGui::GetWindowSize();
    return Eigen::Vector2i(static_cast<int>(size.x), static_cast<int>(size.y));
  });
  imgui_.def("get_window_width", &ImGui::GetWindowWidth);
  imgui_.def("get_window_height", &ImGui::GetWindowHeight);

  // Window manipulation
  imgui_.def(
    "set_next_window_pos",
    [](const Eigen::Vector2i& pos, int cond, const Eigen::Vector2i& pivot) { ImGui::SetNextWindowPos(ImVec2(pos[0], pos[1]), cond, ImVec2(pivot[0], pivot[1])); },
    "",
    py::arg("pos"),
    py::arg("cond") = 0,
    py::arg("pivot") = Eigen::Vector2i(0, 0));
  imgui_
    .def("set_next_window_size", [](const Eigen::Vector2i& size, int cond) { ImGui::SetNextWindowSize(ImVec2(size[0], size[1]), cond); }, "", py::arg("size"), py::arg("cond") = 0);
  imgui_.def(
    "set_next_window_size_constraints",
    [](const Eigen::Vector2i& size_min, const Eigen::Vector2i& size_max) {
      ImGui::SetNextWindowSizeConstraints(ImVec2(size_min[0], size_min[1]), ImVec2(size_max[0], size_max[1]));
    },
    "",
    py::arg("size_min"),
    py::arg("size_max"));
  imgui_.def("set_next_window_content_size", [](const Eigen::Vector2i& size) { ImGui::SetNextWindowContentSize(ImVec2(size[0], size[1])); }, "", py::arg("size"));
  imgui_.def("set_next_window_collapsed", &ImGui::SetNextWindowCollapsed, py::arg("collapsed"), py::arg("cond") = 0);
  imgui_.def("set_next_window_focus", &ImGui::SetNextWindowFocus);
  imgui_.def("set_next_window_scroll", [](const Eigen::Vector2i& scroll) { ImGui::SetNextWindowScroll(ImVec2(scroll[0], scroll[1])); }, py::arg("scroll"));
  imgui_.def("set_next_window_bg_alpha", &ImGui::SetNextWindowBgAlpha, py::arg("alpha"));
  imgui_.def("set_window_pos", [](const Eigen::Vector2i& pos, int cond) { ImGui::SetWindowPos(ImVec2(pos[0], pos[1]), cond); }, py::arg("pos"), py::arg("cond") = 0);
  imgui_.def("set_window_size", [](const Eigen::Vector2i& size, int cond) { ImGui::SetWindowSize(ImVec2(size[0], size[1]), cond); }, py::arg("size"), py::arg("cond") = 0);
  imgui_.def("set_window_collapsed", [](bool collapsed, int cond) { ImGui::SetWindowCollapsed(collapsed, cond); }, py::arg("collapsed"), py::arg("cond") = 0);
  imgui_.def("set_window_focus", [] { ImGui::SetWindowFocus(); });
  imgui_.def("set_window_font_scale", &ImGui::SetWindowFontScale, py::arg("scale"));
  imgui_.def(
    "set_window_pos",
    [](const std::string& name, const Eigen::Vector2i& pos, int cond) { ImGui::SetWindowPos(name.c_str(), ImVec2(pos[0], pos[1]), cond); },
    py::arg("name"),
    py::arg("pos"),
    py::arg("cond") = 0);
  imgui_.def(
    "set_window_size",
    [](const std::string& name, const Eigen::Vector2i& size, int cond) { ImGui::SetWindowSize(name.c_str(), ImVec2(size[0], size[1]), cond); },
    py::arg("name"),
    py::arg("size"),
    py::arg("cond") = 0);
  imgui_.def(
    "set_window_collapsed",
    [](const std::string& name, bool collapsed, int cond) { ImGui::SetWindowCollapsed(name.c_str(), collapsed, cond); },
    py::arg("name"),
    py::arg("collapsed"),
    py::arg("cond") = 0);
  imgui_.def("set_window_focus", [](const std::string& name) { ImGui::SetWindowFocus(name.c_str()); }, py::arg("name"));

  // Content region
  imgui_.def("get_content_region_avail", [] {
    ImVec2 size = ImGui::GetContentRegionAvail();
    return Eigen::Vector2f(size.x, size.y);
  });
  imgui_.def("get_content_region_max", [] {
    ImVec2 size = ImGui::GetContentRegionMax();
    return Eigen::Vector2f(size.x, size.y);
  });
  imgui_.def("get_window_content_region_min", [] {
    ImVec2 size = ImGui::GetWindowContentRegionMin();
    return Eigen::Vector2f(size.x, size.y);
  });
  imgui_.def("get_window_content_region_max", [] {
    ImVec2 size = ImGui::GetWindowContentRegionMax();
    return Eigen::Vector2f(size.x, size.y);
  });

  // Parameters stacks (shared)
  imgui_.def("push_button_repeat", &ImGui::PushButtonRepeat, py::arg("repeat"));
  imgui_.def("pop_button_repeat", &ImGui::PopButtonRepeat);

  // Parameters stacks (current window)
  imgui_.def("push_item_width", &ImGui::PushItemWidth, py::arg("item_width"));
  imgui_.def("pop_item_width", &ImGui::PopItemWidth);
  imgui_.def("set_next_item_width", &ImGui::SetNextItemWidth, py::arg("item_width"));
  imgui_.def("calc_item_width", &ImGui::CalcItemWidth);
  imgui_.def("push_text_wrap_pos", &ImGui::PushTextWrapPos, py::arg("wrap_local_pos_x") = 0.0f);
  imgui_.def("pop_text_wrap_pos", &ImGui::PopTextWrapPos);

  // Cursor / Layout
  imgui_.def("separator", &ImGui::Separator);
  imgui_.def("same_line", &ImGui::SameLine, "", py::arg("offset_from_start_x") = 0.0, py::arg("spacing") = -1.0);
  imgui_.def("newline", &ImGui::NewLine);
  imgui_.def("spacing", &ImGui::Spacing);
  imgui_.def("dummy", [](const Eigen::Vector2i& size) { ImGui::Dummy(ImVec2(size[0], size[1])); }, py::arg("size"));
  imgui_.def("indent", &ImGui::Indent, py::arg("indent_w") = 0.0f);
  imgui_.def("unindent", &ImGui::Unindent, py::arg("indent_w") = 0.0f);
  imgui_.def("begin_group", &ImGui::BeginGroup);
  imgui_.def("end_group", &ImGui::EndGroup);
  imgui_.def("get_cursor_pos", [] {
    ImVec2 pos = ImGui::GetCursorPos();
    return Eigen::Vector2f(pos.x, pos.y);
  });
  imgui_.def("get_cursor_pos_x", &ImGui::GetCursorPosX);
  imgui_.def("get_cursor_pos_y", &ImGui::GetCursorPosY);
  imgui_.def("set_cursor_pos", [](const Eigen::Vector2f& pos) { ImGui::SetCursorPos(ImVec2(pos[0], pos[1])); }, py::arg("pos"));
  imgui_.def("set_cursor_pos_x", &ImGui::SetCursorPosX, py::arg("x"));
  imgui_.def("set_cursor_pos_y", &ImGui::SetCursorPosY, py::arg("y"));
  imgui_.def("get_cursor_screen_pos", [] {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    return Eigen::Vector2f(pos.x, pos.y);
  });
  imgui_.def("set_cursor_screen_pos", [](const Eigen::Vector2f& pos) { ImGui::SetCursorScreenPos(ImVec2(pos[0], pos[1])); }, py::arg("pos"));
  imgui_.def("align_text_to_frame_padding", &ImGui::AlignTextToFramePadding);
  imgui_.def("get_text_line_height", &ImGui::GetTextLineHeight);
  imgui_.def("get_text_line_height_with_spacing", &ImGui::GetTextLineHeightWithSpacing);
  imgui_.def("get_frame_height", &ImGui::GetFrameHeight);
  imgui_.def("get_frame_height_with_spacing", &ImGui::GetFrameHeightWithSpacing);

  // ID stack/scopes
  imgui_.def("push_id", [](const std::string& str_id) { ImGui::PushID(str_id.c_str()); }, py::arg("str_id"));
  imgui_.def("pop_id", &ImGui::PopID);
  imgui_.def("get_id", [](const std::string& name) { return ImGui::GetID(name.c_str()); }, py::arg("name"));

  // Widgets: Text
  imgui_.def("text_unformatted", [](const std::string& text) { ImGui::TextUnformatted(text.c_str()); }, py::arg("text"));
  imgui_.def("text", [](const std::string& text) { ImGui::Text("%s", text.c_str()); }, py::arg("text"));
  imgui_.def(
    "text_colored",
    [](const Eigen::Vector4f& col, const std::string& text) { ImGui::TextColored(ImVec4(col[0], col[1], col[2], col[3]), "%s", text.c_str()); },
    py::arg("col"),
    py::arg("text"));
  imgui_.def("text_disabled", [](const std::string& text) { ImGui::TextDisabled("%s", text.c_str()); }, py::arg("text"));
  imgui_.def("text_wrapped", [](const std::string& text) { ImGui::TextWrapped("%s", text.c_str()); }, py::arg("text"));
  imgui_.def("label_text", [](const std::string& label, const std::string& text) { ImGui::LabelText(label.c_str(), "%s", text.c_str()); }, py::arg("label"), py::arg("text"));
  imgui_.def("bullet_text", [](const std::string& text) { ImGui::BulletText("%s", text.c_str()); }, py::arg("text"));

  // Widgets: Main
  imgui_.def(
    "button",
    [](const std::string& label, const Eigen::Vector2i& size) { return ImGui::Button(label.c_str(), ImVec2(size[0], size[1])); },
    "",
    py::arg("label"),
    py::arg("size") = Eigen::Vector2i(0, 0));
  imgui_.def("small_button", [](const std::string& label) { return ImGui::SmallButton(label.c_str()); }, "", py::arg("label"));
  imgui_.def(
    "invisible_button",
    [](const std::string& id, const Eigen::Vector2i& size, int flags) { return ImGui::InvisibleButton(id.c_str(), ImVec2(size[0], size[1]), flags); },
    "",
    py::arg("id"),
    py::arg("size"),
    py::arg("flags") = 0);
  imgui_.def("arrow_button", ImGui::ArrowButton, py::arg("id"), py::arg("dir"));
  imgui_.def("checkbox", [](const std::string& label, bool v) { return std::make_tuple(ImGui::Checkbox(label.c_str(), &v), v); }, py::arg("label"), py::arg("v"));
  imgui_.def(
    "checkbox_flags",
    [](const std::string& label, int flags, int flags_value) { return std::make_tuple(ImGui::CheckboxFlags(label.c_str(), &flags, flags_value), flags); },
    py::arg("label"),
    py::arg("flags"),
    py::arg("flags_value"));
  imgui_.def("radio_button", [](const std::string& label, bool active) { return ImGui::RadioButton(label.c_str(), active); }, py::arg("label"), py::arg("active"));
  imgui_.def(
    "radio_button_int",
    [](const std::string& label, int v, int v_button) { return std::make_tuple(ImGui::RadioButton(label.c_str(), &v, v_button), v); },
    py::arg("label"),
    py::arg("v"),
    py::arg("v_button"));
  imgui_.def(
    "progress_bar",
    [](float fraction, const Eigen::Vector2i& size, const std::string& overlay) { ImGui::ProgressBar(fraction, ImVec2(size[0], size[1]), overlay.c_str()); },
    py::arg("fraction"),
    py::arg("size") = Eigen::Vector2i(-FLT_MIN, 0),
    py::arg("overlay") = "");
  imgui_.def("bullet", &ImGui::Bullet);

  // Widgets: Images
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
  imgui_.def(
    "image_button",
    [](
      const std::string& str_id,
      const std::shared_ptr<glk::Texture>& texture,
      const Eigen::Vector2i& size,
      const Eigen::Vector2f& uv0,
      const Eigen::Vector2f& uv1,
      const Eigen::Vector4f& bg_col,
      const Eigen::Vector4f& tint_col) {
      return ImGui::ImageButton(
        str_id.c_str(),
        reinterpret_cast<void*>(texture->id()),
        ImVec2(size[0], size[1]),
        ImVec2(uv0[0], uv0[1]),
        ImVec2(uv1[0], uv1[1]),
        ImVec4(bg_col[0], bg_col[1], bg_col[2], bg_col[3]),
        ImVec4(tint_col[0], tint_col[1], tint_col[2], tint_col[3]));
    },
    "",
    py::arg("str_id"),
    py::arg("texture"),
    py::arg("size"),
    py::arg("uv0") = Eigen::Vector2f(0.0, 0.0),
    py::arg("uv1") = Eigen::Vector2f(1.0, 1.0),
    py::arg("bg_col") = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f),
    py::arg("tint_col") = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));

  // Widgets: Combo Box (Dropdown)
  imgui_.def(
    "begin_combo",
    [](const std::string& label, const std::string& preview_value, int flags) { return ImGui::BeginCombo(label.c_str(), preview_value.c_str(), flags); },
    "",
    py::arg("label"),
    py::arg("preview_value"),
    py::arg("flags") = 0);
  imgui_.def("end_combo", &ImGui::EndCombo);
  imgui_.def(
    "combo",
    [](const std::string& label, int current_item, const std::vector<std::string>& items, int popup_max_height_in_items) {
      std::vector<const char*> item_cstrs;
      for (const auto& item : items) {
        item_cstrs.push_back(item.c_str());
      }
      return std::make_tuple(ImGui::Combo(label.c_str(), &current_item, item_cstrs.data(), static_cast<int>(item_cstrs.size()), popup_max_height_in_items), current_item);
    },
    "",
    py::arg("label"),
    py::arg("current_item"),
    py::arg("items"),
    py::arg("popup_max_height_in_items") = -1);

  // Widgets: Drag Sliders
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
  imgui_.def(
    "drag_float2",
    [](const std::string& label, const Eigen::Vector2f& v, float v_speed, float v_min, float v_max, const std::string& format, float power) {
      Eigen::Vector2f val = v;
      const bool changed = ImGui::DragFloat2(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str(), power);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1.0f,
    py::arg("v_min") = 0.0f,
    py::arg("v_max") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);
  imgui_.def(
    "drag_float3",
    [](const std::string& label, const Eigen::Vector3f& v, float v_speed, float v_min, float v_max, const std::string& format, float power) {
      Eigen::Vector3f val = v;
      const bool changed = ImGui::DragFloat3(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str(), power);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1.0f,
    py::arg("v_min") = 0.0f,
    py::arg("v_max") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);
  imgui_.def(
    "drag_float4",
    [](const std::string& label, const Eigen::Vector4f& v, float v_speed, float v_min, float v_max, const std::string& format, float power) {
      Eigen::Vector4f val = v;
      const bool changed = ImGui::DragFloat4(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str(), power);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1.0f,
    py::arg("v_min") = 0.0f,
    py::arg("v_max") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);
  imgui_.def(
    "drag_float_range2",
    [](const std::string& label, const Eigen::Vector2f& v, float v_speed, float v_min, float v_max, const std::string& format, const std::string& format_max, float power) {
      Eigen::Vector2f val = v;
      const bool changed = ImGui::DragFloatRange2(label.c_str(), &val[0], &val[1], v_speed, v_min, v_max, format.c_str(), format_max.c_str(), power);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1.0f,
    py::arg("v_min") = 0.0f,
    py::arg("v_max") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("format_max") = "%.3f",
    py::arg("power") = 1.0f);

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
    "drag_int2",
    [](const std::string& label, const Eigen::Vector2i& v, int v_speed, int v_min, int v_max, const std::string& format) {
      Eigen::Vector2i val = v;
      const bool changed = ImGui::DragInt2(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str());
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1,
    py::arg("v_min") = 0,
    py::arg("v_max") = 0,
    py::arg("format") = "%d");
  imgui_.def(
    "drag_int3",
    [](const std::string& label, const Eigen::Vector3i& v, int v_speed, int v_min, int v_max, const std::string& format) {
      Eigen::Vector3i val = v;
      const bool changed = ImGui::DragInt3(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str());
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1,
    py::arg("v_min") = 0,
    py::arg("v_max") = 0,
    py::arg("format") = "%d");
  imgui_.def(
    "drag_int4",
    [](const std::string& label, const Eigen::Vector4i& v, int v_speed, int v_min, int v_max, const std::string& format) {
      Eigen::Vector4i val = v;
      const bool changed = ImGui::DragInt4(label.c_str(), val.data(), v_speed, v_min, v_max, format.c_str());
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1,
    py::arg("v_min") = 0,
    py::arg("v_max") = 0,
    py::arg("format") = "%d");
  imgui_.def(
    "drag_int_range2",
    [](const std::string& label, const Eigen::Vector2i& v, int v_speed, int v_min, int v_max, const std::string& format, const std::string& format_max) {
      Eigen::Vector2i val = v;
      const bool changed = ImGui::DragIntRange2(label.c_str(), &val[0], &val[1], v_speed, v_min, v_max, format.c_str(), format_max.c_str());
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_speed") = 1,
    py::arg("v_min") = 0,
    py::arg("v_max") = 0,
    py::arg("format") = "%d",
    py::arg("format_max") = "%d");

  // Widgets: Regular Sliders
  imgui_.def(
    "slider_float",
    [](const std::string& label, float v, float v_min, float v_max, const std::string& format, float power) {
      return std::make_tuple(ImGui::SliderFloat(label.c_str(), &v, v_min, v_max, format.c_str(), power), v);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);
  imgui_.def(
    "slider_float2",
    [](const std::string& label, const Eigen::Vector2f& v, float v_min, float v_max, const std::string& format, float power) {
      Eigen::Vector2f val = v;
      const bool changed = ImGui::SliderFloat2(label.c_str(), val.data(), v_min, v_max, format.c_str(), power);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%.3f",
    py::arg("power") = 1.0f);
  imgui_.def(
    "slider_float3",
    [](const std::string& label, const Eigen::Vector3f& v, float v_min, float v_max, const std::string& format, int flags) {
      Eigen::Vector3f val = v;
      const bool changed = ImGui::SliderFloat3(label.c_str(), val.data(), v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_float4",
    [](const std::string& label, const Eigen::Vector4f& v, float v_min, float v_max, const std::string& format, int flags) {
      Eigen::Vector4f val = v;
      const bool changed = ImGui::SliderFloat4(label.c_str(), val.data(), v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_angle",
    [](const std::string& label, float v_rad, float v_degrees_min, float v_degrees_max, const std::string& format, int flags) {
      float val = v_rad;
      const bool changed = ImGui::SliderAngle(label.c_str(), &val, v_degrees_min, v_degrees_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v_rad"),
    py::arg("v_degrees_min") = -360.0f,
    py::arg("v_degrees_max") = 360.0f,
    py::arg("format") = "%.0f deg",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_int",
    [](const std::string& label, int v, int v_min, int v_max, const std::string& format, int flags) {
      int val = v;
      const bool changed = ImGui::SliderInt(label.c_str(), &val, v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%d",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_int2",
    [](const std::string& label, const Eigen::Vector2i& v, int v_min, int v_max, const std::string& format, int flags) {
      Eigen::Vector2i val = v;
      const bool changed = ImGui::SliderInt2(label.c_str(), val.data(), v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%d",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_int3",
    [](const std::string& label, const Eigen::Vector3i& v, int v_min, int v_max, const std::string& format, int flags) {
      Eigen::Vector3i val = v;
      const bool changed = ImGui::SliderInt3(label.c_str(), val.data(), v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%d",
    py::arg("flags") = 0);
  imgui_.def(
    "slider_int4",
    [](const std::string& label, const Eigen::Vector4i& v, int v_min, int v_max, const std::string& format, int flags) {
      Eigen::Vector4i val = v;
      const bool changed = ImGui::SliderInt4(label.c_str(), val.data(), v_min, v_max, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("v_min"),
    py::arg("v_max"),
    py::arg("format") = "%d",
    py::arg("flags") = 0);

  // Widgets: Input with Keyboard
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
  imgui_.def(
    "input_text_multiline",
    [](const std::string& label, const std::string& text, const Eigen::Vector2i& size, int flags, int buffer_size) {
      std::vector<char> buffer(buffer_size, 0);
      std::copy(text.begin(), text.end(), buffer.begin());
      const bool changed = ImGui::InputTextMultiline(label.c_str(), buffer.data(), buffer_size, ImVec2(size[0], size[1]), flags);
      return std::make_pair(changed, std::string(buffer.data()));
    },
    "",
    py::arg("label"),
    py::arg("text"),
    py::arg("size") = Eigen::Vector2i(0, 0),
    py::arg("flags") = 0,
    py::arg("buffer_size") = 256);
  imgui_.def(
    "input_text_with_hint",
    [](const std::string& label, const std::string& hint, const std::string& text, int flags, int buffer_size) {
      std::vector<char> buffer(buffer_size, 0);
      std::copy(text.begin(), text.end(), buffer.begin());
      const bool changed = ImGui::InputTextWithHint(label.c_str(), hint.c_str(), buffer.data(), buffer_size, flags);
      return std::make_pair(changed, std::string(buffer.data()));
    },
    "",
    py::arg("label"),
    py::arg("hint"),
    py::arg("text"),
    py::arg("flags") = 0,
    py::arg("buffer_size") = 256);
  imgui_.def(
    "input_float",
    [](const std::string& label, float v, float step, float step_fast, const std::string& format, int flags) {
      float val = v;
      const bool changed = ImGui::InputFloat(label.c_str(), &val, step, step_fast, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("step") = 0.0f,
    py::arg("step_fast") = 0.0f,
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "input_float2",
    [](const std::string& label, const Eigen::Vector2f& v, const std::string& format, int flags) {
      Eigen::Vector2f val = v;
      const bool changed = ImGui::InputFloat2(label.c_str(), val.data(), format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "input_float3",
    [](const std::string& label, const Eigen::Vector3f& v, const std::string& format, int flags) {
      Eigen::Vector3f val = v;
      const bool changed = ImGui::InputFloat3(label.c_str(), val.data(), format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "input_float4",
    [](const std::string& label, const Eigen::Vector4f& v, const std::string& format, int flags) {
      Eigen::Vector4f val = v;
      const bool changed = ImGui::InputFloat4(label.c_str(), val.data(), format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("format") = "%.3f",
    py::arg("flags") = 0);
  imgui_.def(
    "input_int",
    [](const std::string& label, int v, int step, int step_fast, int flags) {
      int val = v;
      const bool changed = ImGui::InputInt(label.c_str(), &val, step, step_fast, flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("step") = 1,
    py::arg("step_fast") = 100,
    py::arg("flags") = 0);
  imgui_.def(
    "input_int2",
    [](const std::string& label, const Eigen::Vector2i& v, int flags) {
      Eigen::Vector2i val = v;
      const bool changed = ImGui::InputInt2(label.c_str(), val.data(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("flags") = 0);
  imgui_.def(
    "input_int3",
    [](const std::string& label, const Eigen::Vector3i& v, int flags) {
      Eigen::Vector3i val = v;
      const bool changed = ImGui::InputInt3(label.c_str(), val.data(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("flags") = 0);
  imgui_.def(
    "input_int4",
    [](const std::string& label, const Eigen::Vector4i& v, int flags) {
      Eigen::Vector4i val = v;
      const bool changed = ImGui::InputInt4(label.c_str(), val.data(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("flags") = 0);
  imgui_.def(
    "input_double",
    [](const std::string& label, double v, double step, double step_fast, const std::string& format, int flags) {
      double val = v;
      const bool changed = ImGui::InputDouble(label.c_str(), &val, step, step_fast, format.c_str(), flags);
      return std::make_pair(changed, val);
    },
    "",
    py::arg("label"),
    py::arg("v"),
    py::arg("step") = 0.0,
    py::arg("step_fast") = 0.0,
    py::arg("format") = "%.6f",
    py::arg("flags") = 0);

  // Widgets: Color Editor/Picker
  imgui_.def(
    "color_edit3",
    [](const std::string& label, const Eigen::Vector3f& color, int flags) {
      Eigen::Vector3f col = color;
      const bool changed = ImGui::ColorEdit3(label.c_str(), col.data(), flags);
      return std::make_pair(changed, col);
    },
    py::arg("label"),
    py::arg("color"),
    py::arg("flags") = 0);
  imgui_.def(
    "color_edit4",
    [](const std::string& label, const Eigen::Vector4f& color, int flags) {
      Eigen::Vector4f col = color;
      const bool changed = ImGui::ColorEdit4(label.c_str(), col.data(), flags);
      return std::make_pair(changed, col);
    },
    py::arg("label"),
    py::arg("color"),
    py::arg("flags") = 0);
  imgui_.def(
    "color_picker3",
    [](const std::string& label, const Eigen::Vector3f& color, int flags) {
      Eigen::Vector3f col = color;
      const bool changed = ImGui::ColorPicker3(label.c_str(), col.data(), flags);
      return std::make_pair(changed, col);
    },
    py::arg("label"),
    py::arg("color"),
    py::arg("flags") = 0);
  imgui_.def(
    "color_picker4",
    [](const std::string& label, const Eigen::Vector4f& color, int flags) {
      Eigen::Vector4f col = color;
      const bool changed = ImGui::ColorPicker4(label.c_str(), col.data(), flags);
      return std::make_pair(changed, col);
    },
    py::arg("label"),
    py::arg("color"),
    py::arg("flags") = 0);
  imgui_.def(
    "color_button",
    [](const std::string& desc_id, const Eigen::Vector4f& color, int flags, const Eigen::Vector2i& size) {
      return ImGui::ColorButton(desc_id.c_str(), ImVec4(color[0], color[1], color[2], color[3]), flags, ImVec2(size[0], size[1]));
    },
    py::arg("desc_id"),
    py::arg("color"),
    py::arg("flags") = 0,
    py::arg("size") = Eigen::Vector2i(0, 0));

  // Widgets: Trees
  imgui_.def("tree_node", [](const std::string& label) { return ImGui::TreeNode(label.c_str()); }, py::arg("label"));
  imgui_.def("tree_push_str", [](const std::string& str_id) { ImGui::TreePush(str_id.c_str()); }, py::arg("str_id"));
  imgui_.def("tree_pop", &ImGui::TreePop);
  imgui_.def("get_tree_node_to_label_spacing", &ImGui::GetTreeNodeToLabelSpacing);
  imgui_.def("collapsing_header", [](const std::string& label, int flags) { return ImGui::CollapsingHeader(label.c_str(), flags); }, py::arg("label"), py::arg("flags") = 0);
  imgui_.def(
    "collapsing_header_bool",
    [](const std::string& label, bool open, int flags) { return std::make_tuple(ImGui::CollapsingHeader(label.c_str(), &open, flags), open); },
    py::arg("label"),
    py::arg("open"),
    py::arg("flags") = 0);
  imgui_.def("set_next_item_open", &ImGui::SetNextItemOpen, py::arg("is_open"), py::arg("cond") = 0);

  // Widgets: Selectables
  imgui_.def(
    "selectable",
    [](const std::string& label, bool selected, int flags, const Eigen::Vector2i& size) { return ImGui::Selectable(label.c_str(), selected, flags, ImVec2(size[0], size[1])); },
    "",
    py::arg("label"),
    py::arg("selected") = false,
    py::arg("flags") = 0,
    py::arg("size") = Eigen::Vector2i(0, 0));

  // Widgets: Menus
  imgui_.def("begin_menu_bar", &ImGui::BeginMenuBar);
  imgui_.def("end_menu_bar", &ImGui::EndMenuBar);
  imgui_.def("begin_main_menu_bar", &ImGui::BeginMainMenuBar);
  imgui_.def("end_main_menu_bar", &ImGui::EndMainMenuBar);
  imgui_.def("begin_menu", [](const std::string& label, bool enabled) { return ImGui::BeginMenu(label.c_str(), enabled); }, py::arg("label"), py::arg("enabled") = true);
  imgui_.def("end_menu", &ImGui::EndMenu);
  imgui_.def(
    "menu_item",
    [](const std::string& label, const std::string& shortcut, bool selected, bool enabled) { return ImGui::MenuItem(label.c_str(), shortcut.c_str(), &selected, enabled); },
    py::arg("label"),
    py::arg("shortcut") = "",
    py::arg("selected") = false,
    py::arg("enabled") = true);

  // Tooltips
  imgui_.def("begin_tooltip", &ImGui::BeginTooltip);
  imgui_.def("end_tooltip", &ImGui::EndTooltip);
  imgui_.def("set_tooltip", [](const std::string& fmt) { ImGui::SetTooltip("%s", fmt.c_str()); }, py::arg("fmt"));

  // Popups: begin/end functions
  imgui_.def("begin_popup", &ImGui::BeginPopup, py::arg("id"), py::arg("flags"));
  imgui_.def("begin_popup_modal", [](const std::string& name, int flags) { return ImGui::BeginPopupModal(name.c_str(), nullptr, flags); }, py::arg("name"), py::arg("flags") = 0);
  imgui_.def("end_popup", &ImGui::EndPopup);

  // Popups: open/close functions
  imgui_.def("open_popup", [](const std::string& name) { ImGui::OpenPopup(name.c_str()); }, py::arg("name"));
  imgui_.def(
    "open_popup_on_item_click",
    [](const std::string& str_id, int mouse_button) { ImGui::OpenPopupOnItemClick(str_id.c_str(), mouse_button); },
    py::arg("str_id"),
    py::arg("mouse_button") = 0);
  imgui_.def("close_current_popup", &ImGui::CloseCurrentPopup);

  // Popups: query functions
  imgui_.def("is_popup_open", [](const std::string& name, int flags) { return ImGui::IsPopupOpen(name.c_str(), flags); }, py::arg("name"), py::arg("flags") = 0);

  // Tables
  imgui_.def("begin_table", &ImGui::BeginTable, py::arg("str_id"), py::arg("column_n"), py::arg("flags") = 0, py::arg("outer_size") = ImVec2(0, 0), py::arg("inner_width") = 0.0f);
  imgui_.def("end_table", &ImGui::EndTable);
  imgui_.def("table_next_row", &ImGui::TableNextRow, py::arg("row_flags") = 0, py::arg("min_row_height") = 0.0f);
  imgui_.def("table_next_column", &ImGui::TableNextColumn);
  imgui_.def("table_set_column_index", &ImGui::TableSetColumnIndex, py::arg("column_index"));
  imgui_.def("table_setup_column", &ImGui::TableSetupColumn, py::arg("label"), py::arg("flags") = 0, py::arg("init_width_or_weight") = 0.0f, py::arg("user_id") = 0);
  imgui_.def("table_setup_scroll_freeze", &ImGui::TableSetupScrollFreeze, py::arg("cols"), py::arg("rows"));
  imgui_.def("table_headers_row", &ImGui::TableHeadersRow);
  imgui_.def("table_header", &ImGui::TableHeader, py::arg("label"));

  // Tab Bars, Tabs
  imgui_.def("begin_tab_bar", &ImGui::BeginTabBar, py::arg("id"), py::arg("flags") = 0);
  imgui_.def("end_tab_bar", &ImGui::EndTabBar);
  imgui_.def("begin_tab_item", [](const char* label) { return ImGui::BeginTabItem(label); }, py::arg("label"));
  imgui_.def("end_tab_item", &ImGui::EndTabItem);
  imgui_.def("tab_item_button", &ImGui::TabItemButton, py::arg("label"), py::arg("flags") = 0);
  imgui_.def("set_tab_item_closed", &ImGui::SetTabItemClosed, py::arg("tab_or_dockspace_id"));

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

  // Focus, Activation
  imgui_.def("set_item_default_focus", &ImGui::SetItemDefaultFocus);
  imgui_.def("set_keyboard_focus_here", &ImGui::SetKeyboardFocusHere, py::arg("offset") = 0);

  // Item/Widgets Utilities and Query Functions
  imgui_.def("is_item_hovered", &ImGui::IsItemHovered, py::arg("flags") = 0);
  imgui_.def("is_item_active", &ImGui::IsItemActive);
  imgui_.def("is_item_focused", &ImGui::IsItemFocused);
  imgui_.def("is_item_clicked", &ImGui::IsItemClicked, py::arg("mouse_button") = 0);
  imgui_.def("is_item_visible", &ImGui::IsItemVisible);
  imgui_.def("is_item_edited", &ImGui::IsItemEdited);
  imgui_.def("is_item_activated", &ImGui::IsItemActivated);
  imgui_.def("is_item_deactivated", &ImGui::IsItemDeactivated);
  imgui_.def("is_item_deactivated_after_edit", &ImGui::IsItemDeactivatedAfterEdit);
  imgui_.def("is_item_toggled_open", &ImGui::IsItemToggledOpen);
  imgui_.def("is_any_item_hovered", &ImGui::IsAnyItemHovered);
  imgui_.def("is_any_item_active", &ImGui::IsAnyItemActive);
  imgui_.def("is_any_item_focused", &ImGui::IsAnyItemFocused);
  imgui_.def("get_item_id", &ImGui::GetItemID);
  imgui_.def("get_item_rect_min", [] {
    auto pos = ImGui::GetItemRectMin();
    return Eigen::Vector2f(pos[0], pos[1]);
  });
  imgui_.def("get_item_rect_max", [] {
    auto pos = ImGui::GetItemRectMax();
    return Eigen::Vector2f(pos[0], pos[1]);
  });
  imgui_.def("get_item_rect_size", [] {
    auto size = ImGui::GetItemRectSize();
    return Eigen::Vector2f(size[0], size[1]);
  });
  imgui_.def("set_item_allow_overlap", &ImGui::SetItemAllowOverlap);

  // Background/Foreground Draw Lists (return raw pointers)
  imgui_.def("get_background_draw_list", [] { return ImGui::GetBackgroundDrawList(); }, py::return_value_policy::reference);
  imgui_.def("get_foreground_draw_list", [] { return ImGui::GetForegroundDrawList(); }, py::return_value_policy::reference);

  // Miscellaneous Utilities
  imgui_.def("is_rect_visible", [](const Eigen::Vector2f& size) {
    ImVec2 sz(size[0], size[1]);
    return ImGui::IsRectVisible(sz);
  });
  imgui_.def("is_rect_visible_pos_size", [](const Eigen::Vector2f& pos, const Eigen::Vector2f& size) {
    ImVec2 p(pos[0], pos[1]);
    ImVec2 sz(size[0], size[1]);
    return ImGui::IsRectVisible(p, sz);
  });
  imgui_.def("get_time", &ImGui::GetTime);
  imgui_.def("get_frame_count", &ImGui::GetFrameCount);

  // Text Utilities
  imgui_.def(
    "calc_text_size",
    [](const std::string& text, float wrap_width, bool hide_text_after_double_hash) {
      auto size = ImGui::CalcTextSize(text.c_str(), nullptr, hide_text_after_double_hash, wrap_width);
      return Eigen::Vector2f(size[0], size[1]);
    },
    py::arg("text"),
    py::arg("wrap_width") = -1.0f,
    py::arg("hide_text_after_double_hash") = false);

  // Inputs Utilities: Keyboard/Mouse/Gamepad
  imgui_.def("is_key_down", [](int key) { return ImGui::IsKeyDown(static_cast<ImGuiKey>(key)); }, py::arg("key"));
  imgui_.def("is_key_pressed", [](int key, bool repeat) { return ImGui::IsKeyPressed(static_cast<ImGuiKey>(key), repeat); }, py::arg("key"), py::arg("repeat") = false);
  imgui_.def("is_key_released", [](int key) { return ImGui::IsKeyReleased(static_cast<ImGuiKey>(key)); }, py::arg("key"));
  imgui_.def("get_key_pressed_amount", &ImGui::GetKeyPressedAmount, py::arg("key"), py::arg("repeat_delay") = 0.0f, py::arg("repeat_rate") = 0.0f);
  imgui_.def("get_key_name", [](int key_index) { return std::string(ImGui::GetKeyName(static_cast<ImGuiKey>(key_index))); }, py::arg("key_index"));

  // Inputs Utilities: Mouse specific
  imgui_.def("is_mouse_down", [](int button) { return ImGui::IsMouseDown(static_cast<ImGuiMouseButton>(button)); }, py::arg("button"));
  imgui_.def(
    "is_mouse_clicked",
    [](int button, bool repeat) { return ImGui::IsMouseClicked(static_cast<ImGuiMouseButton>(button), repeat); },
    py::arg("button"),
    py::arg("repeat") = false);
  imgui_.def("is_mouse_released", [](int button) { return ImGui::IsMouseReleased(static_cast<ImGuiMouseButton>(button)); }, py::arg("button"));
  imgui_.def("is_mouse_double_clicked", [](int button) { return ImGui::IsMouseDoubleClicked(static_cast<ImGuiMouseButton>(button)); }, py::arg("button"));
  imgui_.def("get_mouse_clicked_count", [](int button) { return ImGui::GetMouseClickedCount(static_cast<ImGuiMouseButton>(button)); }, py::arg("button"));
  imgui_.def(
    "is_mouse_hovering_rect",
    [](const Eigen::Vector2f& r_min, const Eigen::Vector2f& r_max, bool clip) {
      ImVec2 min(r_min[0], r_min[1]);
      ImVec2 max(r_max[0], r_max[1]);
      return ImGui::IsMouseHoveringRect(min, max, clip);
    },
    py::arg("r_min"),
    py::arg("r_max"),
    py::arg("clip") = true);
  imgui_.def(
    "is_mouse_pos_valid",
    [](const Eigen::Vector2f& mouse_pos) {
      ImVec2 pos(mouse_pos[0], mouse_pos[1]);
      return ImGui::IsMousePosValid(&pos);
    },
    py::arg("mouse_pos") = Eigen::Vector2f(-FLT_MAX, -FLT_MAX));
  imgui_.def("is_any_mouse_down", &ImGui::IsAnyMouseDown);
  imgui_.def("get_mouse_pos", [] {
    auto pos = ImGui::GetMousePos();
    return Eigen::Vector2f(pos[0], pos[1]);
  });
  imgui_.def("get_mouse_pos_on_opening_current_popup", [] {
    auto pos = ImGui::GetMousePosOnOpeningCurrentPopup();
    return Eigen::Vector2f(pos[0], pos[1]);
  });
  imgui_.def(
    "is_mouse_dragging",
    [](int button, float lock_threshold) { return ImGui::IsMouseDragging(static_cast<ImGuiMouseButton>(button), lock_threshold); },
    py::arg("button"),
    py::arg("lock_threshold") = -1.0f);
  imgui_.def(
    "get_mouse_drag_delta",
    [](int button, float lock_threshold) {
      auto pos = ImGui::GetMouseDragDelta(static_cast<ImGuiMouseButton>(button), lock_threshold);
      return Eigen::Vector2f(pos[0], pos[1]);
    },
    py::arg("button") = 0,
    py::arg("lock_threshold") = -1.0f);
  imgui_.def("reset_mouse_drag_delta", [](int button) { ImGui::ResetMouseDragDelta(static_cast<ImGuiMouseButton>(button)); }, py::arg("button") = 0);

  // Clipboard Utilities
  imgui_.def("get_clipboard_text", [] { return std::string(ImGui::GetClipboardText()); });
  imgui_.def("set_clipboard_text", [](const std::string& text) { ImGui::SetClipboardText(text.c_str()); }, py::arg("text"));

  //
  imgui_.def("get_window_draw_list", [] {
    auto draw_list = ImGui::GetWindowDrawList();
    return std::shared_ptr<ImDrawList>(draw_list, [](ImDrawList* ptr) {});
  });
}