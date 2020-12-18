#include <guik/viewer/viewer_ui.hpp>

#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_space_iridecent_lighting.hpp>

#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>

namespace guik {

/**
 * DisplaySettingWindow
 */
class LightViewer::ViewerUI::DisplaySettingWindow {
public:
  DisplaySettingWindow(guik::LightViewer* viewer) : viewer(viewer) {
    show_window = false;
    colormap_mode = static_cast<int>(glk::COLORMAP::TURBO);
    colormap_axis = 2;
    auto_range = false;

    effect_mode = 0;
  }

  ~DisplaySettingWindow() {}

  void menu_item() {
    ImGui::MenuItem("Shader Setting", nullptr, &show_window);
  }

  void draw_ui() {
    if(!show_window) {
      return;
    }

    ImGui::Begin("display setting", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    auto colormap_modes = glk::colormap_names();
    if(ImGui::Combo("Colormap mode", &colormap_mode, colormap_modes.data(), colormap_modes.size())) {
      viewer->set_colormap(static_cast<glk::COLORMAP>(colormap_mode));
    }

    std::vector<const char*> colormap_axes = {"X_AXIS", "Y_AXIS", "Z_AXIS"};
    if(ImGui::Combo("Colormap axis", &colormap_axis, colormap_axes.data(), colormap_axes.size())) {
      Eigen::Vector3f axis = Eigen::Vector3f::Zero();
      axis[static_cast<int>(colormap_axis)] = 1.0f;
      viewer->shader_setting().add("colormap_axis", axis);
    }

    auto point_size = viewer->shader_setting().get<float>("point_size");
    if(!point_size) {
      point_size = 10.0f;
    }
    if(ImGui::DragFloat("Point size", point_size.get_ptr(), 0.1f)) {
      viewer->shader_setting().add("point_size", *point_size);
    }

    auto z_range = viewer->shader_setting().get<Eigen::Vector2f>("z_range");
    if(!z_range) {
      z_range = Eigen::Vector2f(-3.0f, 5.0f);
    }

    if(ImGui::DragFloatRange2("z_range", z_range->data(), z_range->data() + 1, 0.1f)) {
      viewer->shader_setting().add("z_range", *z_range);
    }

    ImGui::SameLine();
    ImGui::Checkbox("Auto", &auto_range);

    if(auto_range) {
      Eigen::Vector3f axis = Eigen::Vector3f::Zero();
      axis[static_cast<int>(colormap_axis)] = 1.0f;
      Eigen::Vector2f range(std::numeric_limits<float>::max(), std::numeric_limits<float>::min());

      for(const auto& drawable : viewer->drawables) {
        auto model_matrix = drawable.second.first->get<Eigen::Matrix4f>("model_matrix");
        if(!model_matrix) {
          continue;
        }

        float p = model_matrix->block<3, 1>(0, 3).dot(axis);
        range[0] = std::min(range[0], p);
        range[1] = std::max(range[1], p);
      }

      viewer->shader_setting().add("z_range", (range + Eigen::Vector2f(-1.0f, 1.0f)).eval());
    }

    ImGui::Separator();
    std::vector<const char*> effect_modes = {"PLAIN", "NAIVE_SSAO", "NORMAL", "SSAO", "SSLI", "IRIDESCENCE"};
    if(ImGui::Combo("Effect", &effect_mode, effect_modes.data(), effect_modes.size())) {
      if(effect_modes[effect_mode] == std::string("PLAIN")) {
        viewer->set_screen_effect(std::make_shared<glk::PlainRendering>());
      } else if(effect_modes[effect_mode] == std::string("NAIVE_SSAO")) {
        viewer->set_screen_effect(std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>());
      } else if(effect_modes[effect_mode] == std::string("NORMAL")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAttributeEstimation>(viewer->canvas_size(), glk::ScreenSpaceAttributeEstimation::BufferType::NORMAL));
      } else if(effect_modes[effect_mode] == std::string("SSAO")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>(viewer->canvas_size()));
      } else if(effect_modes[effect_mode] == std::string("SSLI")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size()));
      } else if(effect_modes[effect_mode] == std::string("IRIDESCENCE")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceIridescenceLighting>(viewer->canvas_size()));
      }
    }

    ImGui::End();
  }

private:
  LightViewer* viewer;

  bool show_window;

  int colormap_mode;
  int colormap_axis;

  bool auto_range;

  int effect_mode;
};

/**
 * DrawableFilterWindow
 */
class LightViewer::ViewerUI::DrawableFilterWindow {
public:
  DrawableFilterWindow(guik::LightViewer* viewer) : viewer(viewer) {
    show_window = false;

    all_allow = 1;
    filter_pattern.resize(128, 0);
    viewer->add_drawable_filter("general_drawable_filter", [this](const std::string& name) { return drawable_filter(name); });
  }

  ~DrawableFilterWindow() {
    viewer->remove_drawable_filter("general_drawable_filter");
  }

  void menu_item() {
    ImGui::MenuItem("Filter", nullptr, &show_window);
  }

  void draw_ui() {
    if(!show_window) {
      return;
    }

    ImGui::Begin("drawable filter", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::RadioButton("All allow", &all_allow, 1);
    ImGui::SameLine();
    ImGui::RadioButton("All deny", &all_allow, 0);

    bool allow = ImGui::Button("Allow");
    ImGui::SameLine();
    bool deny = ImGui::Button("Deny");
    ImGui::SameLine();
    ImGui::InputTextWithHint("##pattern", "Filter pattern regex", filter_pattern.data(), filter_pattern.size());
    std::regex filter_regex(std::string(filter_pattern.data()));

    std::vector<std::string> filtered;
    for(auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++) {
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

private:
  bool drawable_filter(const std::string& drawable_name) const {
    for(const auto& filter : drawable_filters) {
      if(std::regex_match(drawable_name, std::get<2>(filter))) {
        return std::get<0>(filter);
      }
    }

    return all_allow;
  }

private:
  LightViewer* viewer;

  bool show_window;

  int all_allow;
  std::vector<char> filter_pattern;
  std::vector<std::tuple<bool, std::string, std::regex>> drawable_filters;
};

/**
 * DrawableEditorWindow
 */
class LightViewer::ViewerUI::DrawableEditorWindow {
public:
  DrawableEditorWindow(guik::LightViewer* viewer) : viewer(viewer) {
    show_window = false;
    search_pattern.resize(128, 0);
  }

  ~DrawableEditorWindow() {}

  void menu_item() {
    ImGui::MenuItem("Editor", nullptr, &show_window);
  }

  void draw_ui() {
    if(!show_window) {
      return;
    }

    ImGui::Begin("drawable editor", &show_window, ImGuiWindowFlags_AlwaysAutoResize);
    if(ImGui::Button("Clear")) {
      std::fill(search_pattern.begin(), search_pattern.end(), 0);
    }
    ImGui::SameLine();
    ImGui::InputTextWithHint("##editor_search_pattern", "Search regex", search_pattern.data(), search_pattern.size());

    std::vector<std::string> candidates;
    std::regex search_regex(".*" + std::string(search_pattern.data()) + ".*");
    for(auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++) {
      if(search_pattern[0] && !std::regex_match(itr->first, search_regex)) {
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
            drawable_control.reset();
            std::fill(search_pattern.begin(), search_pattern.end(), 0);
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

      if(!drawable_control || drawable_control->model_name() != drawable_name) {
        drawable_control.reset(new ModelControl(drawable_name));

        auto current_matrix = shader_setting->get<Eigen::Matrix4f>("model_matrix");
        if(current_matrix) {
          drawable_control->set_model_matrix(*current_matrix);
        }
      }

      auto canvas_size = viewer->canvas_size();
      drawable_control->draw_gizmo_ui();
      drawable_control->draw_gizmo(0, 0, canvas_size[0], canvas_size[1], viewer->view_matrix(), viewer->projection_matrix());
      shader_setting->add("model_matrix", drawable_control->model_matrix());

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

private:
  LightViewer* viewer;

  bool show_window;

  std::vector<char> search_pattern;
  std::pair<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>> selected_drawable;
  std::unique_ptr<ModelControl> drawable_control;
};


/**
 * CameraSettingWindow
 */
class LightViewer::ViewerUI::CameraSettingWindow {
public:
  CameraSettingWindow(guik::LightViewer* viewer) : viewer(viewer) {
    show_window = false;
    view_mode = 0;
  }

  ~CameraSettingWindow() {}

  void menu_item() {
    ImGui::MenuItem("Setting", nullptr, &show_window);
  }

  void draw_ui() {
    if(!show_window) {
      return;
    }

    ImGui::Begin("camera", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

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

    viewer->get_projection_control()->draw_ui();

    ImGui::End();
  }

private:
  LightViewer* viewer;

  bool show_window;
  int view_mode;
};

/**
 * ViewerUI::ViewerUI
 */
LightViewer::ViewerUI::ViewerUI(guik::LightViewer* viewer) : viewer(viewer) {
  display_setting_window.reset(new DisplaySettingWindow(viewer));
  drawable_filter_window.reset(new DrawableFilterWindow(viewer));
  drawable_editor_window.reset(new DrawableEditorWindow(viewer));
  camera_setting_window.reset(new CameraSettingWindow(viewer));
}
/**
 * ViewerUI::ViewerUI
 */

LightViewer::ViewerUI::~ViewerUI() {
}

/**
 * ViewerUI::draw_ui
 */
bool LightViewer::ViewerUI::draw_ui() {
  if(draw_main_menu_bar()) {
    return false;
  }

  display_setting_window->draw_ui();
  drawable_filter_window->draw_ui();
  drawable_editor_window->draw_ui();
  camera_setting_window->draw_ui();

  return true;
}

/**
 * ViewerUI::draw_main_menu_bar
 */
bool LightViewer::ViewerUI::draw_main_menu_bar() {
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

  if(ImGui::BeginMenu("Display")) {
    display_setting_window->menu_item();
    if(ImGui::MenuItem("Show Info Window")) {
      viewer->show_info_window();
    }
    ImGui::EndMenu();
  }

  if(ImGui::BeginMenu("Drawable")) {
    drawable_filter_window->menu_item();
    drawable_editor_window->menu_item();
    ImGui::EndMenu();
  }

  if(ImGui::BeginMenu("Camera")) {
    camera_setting_window->menu_item();
    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();

  return hide_menu;
}



}