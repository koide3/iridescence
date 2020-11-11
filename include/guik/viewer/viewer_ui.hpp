#ifndef LIGHT_VIEWER_UI_HPP
#define LIGHT_VIEWER_UI_HPP

#include <regex>

#include <imgui.h>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>

namespace guik {

class LightViewer::ViewerUI {
public:
  ViewerUI(guik::LightViewer* viewer) : viewer(viewer) {
    show_drawable_filter_window = false;
    show_drawable_editor_window = false;
    show_camera_setting_window = false;

    filter_all_allow = 1;
    filter_pattern.resize(128, 0);
    filter_search_pattern.resize(128, 0);
    editor_search_pattern.resize(128, 0);
    viewer->add_drawable_filter("general_drawable_filter", [this](const std::string& name) { return drawable_filter(name); });

    view_mode = 0;
  }
  ~ViewerUI() {
    viewer->remove_drawable_filter("general_drawable_filter");
  }

  bool draw_ui() {
    bool hide_menu = draw_main_menu_bar();

    draw_drawable_filter_window();
    draw_drawable_editor_window();
    draw_camera_window();

    return !hide_menu;
  }

  bool draw_main_menu_bar() {
    bool hide_menu = false;
    ImGui::BeginMainMenuBar();
    if(ImGui::BeginMenu("File")) {
      if(ImGui::MenuItem("Hide menu")) {
        hide_menu = true;
      }

      if(ImGui::MenuItem("Quit")) {
        viewer->close();
      }
      ImGui::EndMenu();
    }

    if(ImGui::BeginMenu("Drawable")) {
      ImGui::MenuItem("Filter", nullptr, &show_drawable_filter_window);
      ImGui::MenuItem("Editor", nullptr, &show_drawable_editor_window);
      ImGui::EndMenu();
    }

    if(ImGui::BeginMenu("Camera")) {
      ImGui::MenuItem("Setting", nullptr, &show_camera_setting_window);
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();

    return hide_menu;
  }

  void draw_drawable_filter_window() {
    if(!show_drawable_filter_window) {
      return;
    }

    ImGui::Begin("drawables", &show_drawable_filter_window, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::RadioButton("All allow", &filter_all_allow, 1);
    ImGui::SameLine();
    ImGui::RadioButton("All deny", &filter_all_allow, 0);

    bool allow = ImGui::Button("Allow");
    ImGui::SameLine();
    bool deny = ImGui::Button("Deny");
    ImGui::SameLine();
    ImGui::InputTextWithHint("##pattern", "Filter pattern regex", filter_pattern.data(), filter_pattern.size());
    std::regex filter_regex(std::string(filter_pattern.data()));

    std::vector<std::string> filtered;
    for(auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++){
      if(filter_pattern[0] && !std::regex_match(itr->first, filter_regex)) {
        continue;
      }
      filtered.push_back(itr->first);
    }

    if(filtered.size()) {
      ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
      ImGui::BeginChild("filtered_list", ImVec2(ImGui::GetWindowContentRegionWidth(), std::min<int>(120, filtered.size() * 20)), false, window_flags);
      for(const auto& name : filtered) {
        ImGui::Text(name.c_str());
      }
      ImGui::EndChild();
    }

    if((allow || deny) && filter_pattern[0]) {
      std::string pattern(filter_pattern.data());
      std::fill(filter_pattern.begin(), filter_pattern.end(), 0);
      drawable_filters.push_back(std::make_tuple(allow, pattern, filter_regex));
    }

    if(!drawable_filters.empty()) {
      ImGui::Separator();
      ImGui::Text("filter regex");

      for(int i = 0; i < drawable_filters.size(); i++) {
        std::string button_name = "Remove##" + std::to_string(i);
        if(ImGui::Button(button_name.c_str())) {
          drawable_filters.erase(drawable_filters.begin() + i);
          continue;
        }

        ImGui::SameLine();
        ImGui::Text("%c:%s", std::get<0>(drawable_filters[i]) ? 'A' : 'D', std::get<1>(drawable_filters[i]).data());
      }
    }

    ImGui::End();
  }

  void draw_drawable_editor_window() {
    if(!show_drawable_editor_window) {
      return;
    }

    ImGui::Begin("drawable edit", &show_drawable_editor_window, ImGuiWindowFlags_AlwaysAutoResize);
    if(ImGui::Button("Clear")) {
      std::fill(editor_search_pattern.begin(), editor_search_pattern.end(), 0);
    }
    ImGui::SameLine();
    ImGui::InputTextWithHint("##editor_search_pattern", "Search regex", editor_search_pattern.data(), editor_search_pattern.size());

    std::vector<std::string> candidates;
    std::regex search_regex(".*" + std::string(editor_search_pattern.data()) + ".*");
    for(auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++) {
      if(editor_search_pattern[0] && !std::regex_match(itr->first, search_regex)) {
        continue;
      }
      candidates.push_back(itr->first);
    }

    if(candidates.size()) {
      ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
      ImGui::BeginChild("candidate_list", ImVec2(ImGui::GetWindowContentRegionWidth(), std::min<int>(120, candidates.size() * 20)), false, window_flags);
      for(const auto& candidate : candidates) {
        std::string select_button_name = "Select##" + candidate;
        if(ImGui::Button(select_button_name.c_str())) {
          auto found = viewer->drawables.find(candidate);
          if(found != viewer->drawables.end()) {
            selected_drawable = *found;
            selected_drawable_control.reset();
            std::fill(editor_search_pattern.begin(), editor_search_pattern.end(), 0);
          }
        }

        std::string remove_button_name = "Remove##" + candidate;
        ImGui::SameLine();
        if(ImGui::Button(remove_button_name.c_str())) {
          auto found = viewer->drawables.find(candidate);
          if(found != viewer->drawables.end()) {
            viewer->drawables.erase(found);
          }
        }

        ImGui::SameLine();
        ImGui::Text(candidate.c_str());
      }
      ImGui::EndChild();
    }

    if(selected_drawable.second.first) {
      const auto& drawable_name = selected_drawable.first;
      const auto& shader_setting = selected_drawable.second.first;

      ImGui::Separator();
      ImGui::Text("Selected:%s", drawable_name.c_str());

      if(!selected_drawable_control || selected_drawable_control->model_name() != drawable_name) {
        selected_drawable_control.reset(new ModelControl(drawable_name));

        auto current_matrix = shader_setting->get<Eigen::Matrix4f>("model_matrix");
        if(current_matrix) {
          selected_drawable_control->set_model_matrix(*current_matrix);
        }
      }

      auto canvas_size = viewer->canvas_size();
      selected_drawable_control->draw_gizmo_ui();
      selected_drawable_control->draw_gizmo(0, 0, canvas_size[0], canvas_size[1], viewer->view_matrix(), viewer->projection_matrix());
      shader_setting->add("model_matrix", selected_drawable_control->model_matrix());

      auto color_mode = shader_setting->get<int>("color_mode");
      if(color_mode) {
        std::vector<const char*> color_modes = {"RAINBOW", "FLAT_COLOR", "VERTEX_COLOR"};
        if(ImGui::Combo("Color mode", color_mode.get_ptr(), color_modes.data(), color_modes.size())) {
          shader_setting->add("color_mode", *color_mode);
        }

        if(color_modes[*color_mode] == std::string("FLAT_COLOR")) {
          auto material_color = shader_setting->get<Eigen::Vector4f>("material_color");
          if(!material_color) {
            material_color = Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f);
          }

          if(ImGui::ColorPicker4("##color_picker", material_color->data())) {
            shader_setting->add("material_color", *material_color);
          }
        }
      }
    }
    ImGui::End();
  }

  void draw_camera_window() {
    if(!show_camera_setting_window) {
      return;
    }

    ImGui::Begin("camera", &show_camera_setting_window, ImGuiWindowFlags_AlwaysAutoResize);

    std::vector<const char*> view_modes = {"Orbit(XY)", "Orbit(XZ)", "TOPDOWN"};
    if(ImGui::Combo("View mode", &view_mode, view_modes.data(), view_modes.size())) {
      if(view_modes[view_mode] == std::string("Orbit(XY)")) {
        viewer->set_camera_control(std::make_shared<OrbitCameraControlXY>());
      }
      if(view_modes[view_mode] == std::string("Orbit(XZ)")) {
        viewer->set_camera_control(std::make_shared<OrbitCameraControlXZ>());
      }
      if(view_modes[view_mode] == std::string("TOPDOWN")) {
        viewer->set_camera_control(std::make_shared<TopDownCameraControl>());
      }
    }

    float fov = viewer->get_projection_control()->get_fov();
    if(ImGui::DragFloat("FOV", &fov, 0.1f, 0.1f, 180.0f)) {
      viewer->get_projection_control()->set_fov(fov);
    }

    ImGui::End();
  }

  bool drawable_filter(const std::string& drawable_name) const {
    for(const auto& filter: drawable_filters) {
      if(std::regex_match(drawable_name, std::get<2>(filter))) {
        return std::get<0>(filter);
      }
    }

    return filter_all_allow;
  }

private:
  guik::LightViewer* viewer;

  // drawable filter
  bool show_drawable_filter_window;
  std::vector<char> filter_pattern;
  std::vector<char> filter_search_pattern;
  int filter_all_allow;
  std::vector<std::tuple<bool, std::string, std::regex>> drawable_filters;

  // drawable editor
  bool show_drawable_editor_window;
  std::vector<char> editor_search_pattern;
  std::pair<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::Ptr>> selected_drawable;
  std::unique_ptr<ModelControl> selected_drawable_control;

  // camera setting
  bool show_camera_setting_window;
  int view_mode;
};
}  // namespace guik

#endif