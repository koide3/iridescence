#include <guik/viewer/viewer_ui.hpp>

#include <fstream>

#include <glk/path.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>
#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>

#include <guik/recent_files.hpp>
#include <guik/camera/basic_projection_control.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>
#include <guik/camera/orbit_camera_control_xz.hpp>
#include <guik/camera/topdown_camera_control.hpp>
#include <guik/camera/arcball_camera_control.hpp>

#include <portable-file-dialogs.h>

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

  void menu_item() { ImGui::MenuItem("Shader Setting", nullptr, &show_window); }

  void draw_ui() {
    if (!show_window) {
      return;
    }

    ImGui::Begin("display setting", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    Eigen::Vector4f clear_color = viewer->get_canvas().clear_color;
    if (ImGui::ColorEdit4("Clear color", clear_color.data())) {
      viewer->get_canvas().clear_color = clear_color;
    }

    auto colormap_modes = glk::colormap_names();
    if (ImGui::Combo("Colormap mode", &colormap_mode, colormap_modes.data(), colormap_modes.size())) {
      viewer->set_colormap(static_cast<glk::COLORMAP>(colormap_mode));
    }

    std::vector<const char*> colormap_axes = {"X_AXIS", "Y_AXIS", "Z_AXIS"};
    if (ImGui::Combo("Colormap axis", &colormap_axis, colormap_axes.data(), colormap_axes.size())) {
      Eigen::Vector3f axis = Eigen::Vector3f::Zero();
      axis[static_cast<int>(colormap_axis)] = 1.0f;
      viewer->shader_setting().add("colormap_axis", axis);
    }

    ImGui::Separator();

    auto point_scale_mode = viewer->shader_setting().get<int>("point_scale_mode");
    if (!point_scale_mode) {
      point_scale_mode = 1;
    }
    if (ImGui::Combo("Point scale mode", &point_scale_mode.value(), "SCREENSPACE\0METRIC\0")) {
      viewer->shader_setting().add("point_scale_mode", *point_scale_mode);
      if (*point_scale_mode == guik::PointScaleMode::METRIC) {
        viewer->shader_setting().set_point_size(0.05f);  // 5 cm
      }
    }

    auto point_shape_mode = viewer->shader_setting().get<int>("point_shape_mode");
    if (!point_shape_mode) {
      point_shape_mode = 1;
    }
    if (ImGui::Combo("Point shape mode", &point_shape_mode.value(), "RECTANGLE\0CIRCLE\0")) {
      viewer->shader_setting().add("point_shape_mode", *point_shape_mode);
    }

    auto point_size = viewer->shader_setting().get<float>("point_size");
    if (!point_size) {
      point_size = 0.025f;
    }
    if (ImGui::DragFloat("Point size", &point_size.value(), 0.001f, 0.0f, 1000.0f)) {
      viewer->shader_setting().add("point_size", *point_size);
    }

    auto z_range = viewer->shader_setting().get<Eigen::Vector2f>("z_range");
    if (!z_range) {
      z_range = Eigen::Vector2f(-3.0f, 5.0f);
    }

    if (ImGui::DragFloatRange2("z_range", z_range->data(), z_range->data() + 1, 0.1f)) {
      viewer->shader_setting().add("z_range", *z_range);
    }

    ImGui::SameLine();
    ImGui::Checkbox("Auto", &auto_range);

    auto cmap_range = viewer->shader_setting().get<Eigen::Vector2f>("cmap_range");
    if (!cmap_range) {
      cmap_range = Eigen::Vector2f(0.0f, 1.0f);
    }

    if (ImGui::DragFloatRange2("cmap_range", cmap_range->data(), cmap_range->data() + 1, 0.01f)) {
      viewer->shader_setting().add("cmap_range", *cmap_range);
    }

    if (auto_range) {
      Eigen::Vector3f axis = Eigen::Vector3f::Zero();
      axis[static_cast<int>(colormap_axis)] = 1.0f;
      Eigen::Vector2f range(std::numeric_limits<float>::max(), std::numeric_limits<float>::min());

      for (const auto& drawable : viewer->drawables) {
        auto model_matrix = drawable.second.first->get<Eigen::Matrix4f>("model_matrix");
        if (!model_matrix) {
          continue;
        }

        float p = model_matrix->block<3, 1>(0, 3).dot(axis);
        range[0] = std::min(range[0], p);
        range[1] = std::max(range[1], p);
      }

      viewer->shader_setting().add("z_range", (range + Eigen::Vector2f(-1.0f, 1.0f)).eval());
    }

    ImGui::Separator();
    std::vector<const char*> effect_modes = {"PLAIN", "NAIVE_SSAO", "NORMAL", "SSAO", "SSLI", "SSLI_SPLAT"};
    if (ImGui::Combo("Effect", &effect_mode, effect_modes.data(), effect_modes.size())) {
      if (effect_modes[effect_mode] == std::string("PLAIN")) {
        viewer->set_screen_effect(std::make_shared<glk::PlainRendering>());
      } else if (effect_modes[effect_mode] == std::string("NAIVE_SSAO")) {
        viewer->set_screen_effect(std::make_shared<glk::NaiveScreenSpaceAmbientOcclusion>());
      } else if (effect_modes[effect_mode] == std::string("NORMAL")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAttributeEstimation>(viewer->canvas_size(), glk::ScreenSpaceAttributeEstimation::BufferType::NORMAL));
      } else if (effect_modes[effect_mode] == std::string("SSAO")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceAmbientOcclusion>(viewer->canvas_size()));
      } else if (effect_modes[effect_mode] == std::string("SSLI")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size()));
      } else if (effect_modes[effect_mode] == std::string("SSLI_SPLAT")) {
        viewer->set_screen_effect(std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size(), true));
      }
    }

    if (effect_modes[effect_mode] == std::string("SSLI")) {
      auto ssli = std::dynamic_pointer_cast<glk::ScreenSpaceLighting>(viewer->get_screen_effect());

      if (ssli) {
        float albedo = ssli->get_albedo();
        float roughness = ssli->get_roughness();

        if (ImGui::DragFloat("albedo", &albedo, 0.01f, 0.0f)) {
          ssli->set_albedo(albedo);
        }

        if (ImGui::DragFloat("roughness", &roughness, 0.01f, 0.0f)) {
          ssli->set_roughness(roughness);
        }

        for (int i = 0; i < ssli->num_lights(); i++) {
          const float max_color_magnitude = 3.0f;
          bool directional = ssli->is_light_directional(i);
          Eigen::Vector3f pos = ssli->get_light_pos(i);
          Eigen::Vector4f color = ssli->get_light_color(i) / max_color_magnitude;

          if (directional) {
            pos.normalize();
          }

          std::string label = "Light_" + std::to_string(i);
          ImGui::Separator();
          ImGui::Text("%s", label.c_str());

          label = "directional##" + std::to_string(i);
          if (ImGui::Checkbox(label.c_str(), &directional)) {
            ssli->set_light_directional(i, directional);
          }

          label = (directional ? "dir##" : "pos##") + std::to_string(i);
          const float speed = directional ? 0.01f : 0.1f;
          if (ImGui::DragFloat3(label.c_str(), pos.data(), speed)) {
            if (directional) {
              pos.normalize();
            }
            ssli->set_light_pos(i, pos);
          }

          label = "color##" + std::to_string(i);
          if (ImGui::ColorEdit4(label.c_str(), color.data())) {
            ssli->set_light_color(i, color * max_color_magnitude);
          }
        }

        ImGui::Separator();
        if (ImGui::Button("add positional")) {
          ssli->set_light(ssli->num_lights(), Eigen::Vector3f::Ones() * 10.0f, Eigen::Vector4f::Ones() * 0.5f);
        }

        if (ImGui::Button("add directional")) {
          ssli->set_directional_light(ssli->num_lights(), -Eigen::Vector3f::UnitZ(), Eigen::Vector4f::Ones() * 0.5f);
        }
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
    viewer->register_drawable_filter("general_drawable_filter", [this](const std::string& name) { return drawable_filter(name); });
  }

  ~DrawableFilterWindow() { viewer->register_drawable_filter("general_drawable_filter"); }

  void menu_item() { ImGui::MenuItem("Filter", nullptr, &show_window); }

  void draw_ui() {
    if (!show_window) {
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
    for (auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++) {
      if (filter_pattern[0] && !std::regex_match(itr->first, filter_regex)) {
        continue;
      }
      filtered.push_back(itr->first);
    }

    if (filtered.size()) {
      ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
      ImGui::BeginChild("filtered_list", ImVec2(ImGui::GetWindowContentRegionWidth(), std::min<int>(120, filtered.size() * 20)), false, window_flags);
      for (const auto& name : filtered) {
        std::string allow_button_name = "Allow##" + name;
        if (ImGui::Button(allow_button_name.c_str())) {
          drawable_filters.push_back(std::make_tuple(true, name, std::regex(name)));
        }
        ImGui::SameLine();
        std::string deny_button_name = "Deny##" + name;
        if (ImGui::Button(deny_button_name.c_str())) {
          drawable_filters.push_back(std::make_tuple(false, name, std::regex(name)));
        }

        ImGui::SameLine();
        ImGui::Text("%s", name.c_str());
      }
      ImGui::EndChild();
    }

    if ((allow || deny) && filter_pattern[0]) {
      std::string pattern(filter_pattern.data());
      std::fill(filter_pattern.begin(), filter_pattern.end(), 0);
      drawable_filters.push_back(std::make_tuple(allow, pattern, filter_regex));
    }

    if (!drawable_filters.empty()) {
      ImGui::Separator();
      ImGui::Text("filter regex");

      for (int i = 0; i < drawable_filters.size(); i++) {
        std::string button_name = "Remove##" + std::to_string(i);
        if (ImGui::Button(button_name.c_str())) {
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
    for (const auto& filter : drawable_filters) {
      if (std::regex_match(drawable_name, std::get<2>(filter))) {
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

  void menu_item() { ImGui::MenuItem("Editor", nullptr, &show_window); }

  void draw_ui() {
    if (!show_window) {
      return;
    }

    ImGui::Begin("drawable editor", &show_window, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::Button("Clear")) {
      std::fill(search_pattern.begin(), search_pattern.end(), 0);
    }
    ImGui::SameLine();
    ImGui::InputTextWithHint("##editor_search_pattern", "Search regex", search_pattern.data(), search_pattern.size());

    std::vector<std::string> candidates;
    std::regex search_regex(".*" + std::string(search_pattern.data()) + ".*");
    for (auto itr = viewer->drawables.begin(); itr != viewer->drawables.end(); itr++) {
      if (search_pattern[0] && !std::regex_match(itr->first, search_regex)) {
        continue;
      }
      candidates.push_back(itr->first);
    }

    if (candidates.size()) {
      ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
      ImGui::BeginChild("candidate_list", ImVec2(ImGui::GetWindowContentRegionWidth(), std::min<int>(120, candidates.size() * 20)), false, window_flags);
      for (const auto& candidate : candidates) {
        std::string select_button_name = "Select##" + candidate;
        if (ImGui::Button(select_button_name.c_str())) {
          auto found = viewer->drawables.find(candidate);
          if (found != viewer->drawables.end()) {
            selected_drawable = *found;
            drawable_control.reset();
            std::fill(search_pattern.begin(), search_pattern.end(), 0);
          }
        }

        std::string remove_button_name = "Remove##" + candidate;
        ImGui::SameLine();
        if (ImGui::Button(remove_button_name.c_str())) {
          auto found = viewer->drawables.find(candidate);
          if (found != viewer->drawables.end()) {
            viewer->drawables.erase(found);
          }
        }

        ImGui::SameLine();
        ImGui::Text("%s", candidate.c_str());
      }
      ImGui::EndChild();
    }

    if (selected_drawable.second.first) {
      const auto& drawable_name = selected_drawable.first;
      const auto& shader_setting = selected_drawable.second.first;

      ImGui::Separator();
      ImGui::Text("Selected:%s", drawable_name.c_str());

      if (!drawable_control || drawable_control->model_name() != drawable_name) {
        drawable_control.reset(new ModelControl(drawable_name));

        auto current_matrix = shader_setting->get<Eigen::Matrix4f>("model_matrix");
        if (current_matrix) {
          drawable_control->set_model_matrix(*current_matrix);
        }
      }

      auto canvas_size = viewer->canvas_size();
      drawable_control->draw_gizmo_ui();
      drawable_control->draw_gizmo(0, 0, canvas_size[0], canvas_size[1], viewer->view_matrix(), viewer->projection_matrix());
      shader_setting->add("model_matrix", drawable_control->model_matrix());

      auto color_mode = shader_setting->get<int>("color_mode");
      if (color_mode) {
        std::vector<const char*> color_modes = {"RAINBOW", "FLAT_COLOR", "VERTEX_COLOR"};
        if (ImGui::Combo("Color mode", &color_mode.value(), color_modes.data(), color_modes.size())) {
          shader_setting->add("color_mode", *color_mode);
        }

        if (color_modes[*color_mode] == std::string("FLAT_COLOR")) {
          auto material_color = shader_setting->get<Eigen::Vector4f>("material_color");
          if (!material_color) {
            material_color = Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f);
          }

          if (ImGui::ColorPicker4("##color_picker", material_color->data())) {
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

    if (ImGui::MenuItem("Save Camera")) {
      save_camera();
    }

    if (ImGui::MenuItem("Load Camera")) {
      load_camera();
    }
  }

  void draw_ui() {
    if (!show_window) {
      return;
    }

    ImGui::Begin("camera", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    std::vector<const char*> view_modes = {"Orbit(XY)", "Orbit(XZ)", "TOPDOWN", "ARCBALL"};
    if (ImGui::Combo("View mode", &view_mode, view_modes.data(), view_modes.size())) {
      if (view_modes[view_mode] == std::string("Orbit(XY)")) {
        viewer->set_camera_control(std::shared_ptr<OrbitCameraControlXY>(new OrbitCameraControlXY()));
      }
      if (view_modes[view_mode] == std::string("Orbit(XZ)")) {
        viewer->set_camera_control(std::shared_ptr<OrbitCameraControlXZ>(new OrbitCameraControlXZ()));
      }
      if (view_modes[view_mode] == std::string("TOPDOWN")) {
        viewer->set_camera_control(std::shared_ptr<TopDownCameraControl>(new TopDownCameraControl()));
      }
      if (view_modes[view_mode] == std::string("ARCBALL")) {
        viewer->set_camera_control(std::shared_ptr<ArcBallCameraControl>(new ArcBallCameraControl()));
      }
    }

    viewer->get_projection_control()->draw_ui();

    ImGui::End();
  }

  void save_camera() {
    guik::RecentFiles recent_files("camera_setting_filename");
    std::string default_path = recent_files.empty() ? "/tmp/camera.config" : recent_files.most_recent();
    std::string filename = pfd::save_file("select the destination path", default_path).result();

    if (filename.empty()) {
      return;
    }
    recent_files.push(filename);

    viewer->save_camera_settings(filename);
  }

  void load_camera() {
    guik::RecentFiles recent_files("camera_setting_filename");
    std::string default_path = recent_files.empty() ? "/tmp/camera.config" : recent_files.most_recent();
    auto filenames = pfd::open_file("select the camera config file", default_path).result();

    if (filenames.empty()) {
      return;
    }
    recent_files.push(filenames.front());

    viewer->load_camera_settings(filenames.front());
  }

private:
  LightViewer* viewer;

  bool show_window;
  int view_mode;
};

/**
 * PlotSettingWindow
 */
class LightViewer::ViewerUI::PlotSettingWindow {
public:
  PlotSettingWindow(guik::LightViewer* viewer) : viewer(viewer) { show_window = false; }

  ~PlotSettingWindow() {}

  void menu_item() {
    // ImGui::MenuItem("Setting", nullptr, &show_window);

    if (ImGui::MenuItem("Fit plots (Ctrl+F)")) {
      viewer->fit_all_plots();
    }

    if (ImGui::MenuItem("Clear plots")) {
      viewer->clear_plots(true);
    }
  }

  void draw_ui() {
    if (!show_window) {
      return;
    }

    /*
    ImGui::Begin("camera", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    std::vector<const char*> view_modes = {"Orbit(XY)", "Orbit(XZ)", "TOPDOWN", "ARCBALL"};
    if (ImGui::Combo("View mode", &view_mode, view_modes.data(), view_modes.size())) {
      if (view_modes[view_mode] == std::string("Orbit(XY)")) {
        viewer->set_camera_control(std::shared_ptr<OrbitCameraControlXY>(new OrbitCameraControlXY()));
      }
      if (view_modes[view_mode] == std::string("Orbit(XZ)")) {
        viewer->set_camera_control(std::shared_ptr<OrbitCameraControlXZ>(new OrbitCameraControlXZ()));
      }
      if (view_modes[view_mode] == std::string("TOPDOWN")) {
        viewer->set_camera_control(std::shared_ptr<TopDownCameraControl>(new TopDownCameraControl()));
      }
      if (view_modes[view_mode] == std::string("ARCBALL")) {
        viewer->set_camera_control(std::shared_ptr<ArcBallCameraControl>(new ArcBallCameraControl()));
      }
    }

    viewer->get_projection_control()->draw_ui();

    ImGui::End();
    */
  }

private:
  LightViewer* viewer;

  bool show_window;
};

class LightViewer::ViewerUI::ImGuiDemoWindows {
public:
  ImGuiDemoWindows(guik::LightViewer* viewer) : viewer(viewer) {
    show_demo_window = false;
    show_metrics_window = false;
    show_about_window = false;
    show_misc_window = false;
  }

  ~ImGuiDemoWindows() {}

  void menu_item() {
    if (ImGui::BeginMenu("ImGui")) {
      if (ImGui::MenuItem("Demo window")) {
        show_demo_window = !show_demo_window;
      }

      if (ImGui::MenuItem("Metrics window")) {
        show_metrics_window = !show_metrics_window;
      }

      if (ImGui::MenuItem("About window")) {
        show_about_window = !show_about_window;
      }

      if (ImGui::MenuItem("Miscellaneous")) {
        show_misc_window = !show_misc_window;
      }

      ImGui::EndMenu();
    }
  }
  void draw_ui() {
    if (show_demo_window) {
      ImGui::ShowDemoWindow(&show_demo_window);
    }

    if (show_metrics_window) {
      ImGui::ShowMetricsWindow(&show_metrics_window);
    }

    if (show_about_window) {
      ImGui::ShowAboutWindow(&show_about_window);
    }

    if (show_misc_window) {
      if (ImGui::Begin("Miscellaneous", &show_misc_window, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::ShowFontSelector("Font Selector");
        ImGui::ShowStyleEditor();
      }
    }
  }

private:
  LightViewer* viewer;
  bool show_demo_window;
  bool show_metrics_window;
  bool show_about_window;
  bool show_misc_window;
};

class LightViewer::ViewerUI::PointPickingWindow {
public:
  PointPickingWindow(guik::LightViewer* viewer) : viewer(viewer) {
    show_window = false;

    clicked_pos_2d.setZero();
    clicked_depth = 0.0f;
    clicked_pos_3d.setZero();
  }

  ~PointPickingWindow() {}

  void menu_item() { ImGui::MenuItem("Point Picking", nullptr, &show_window); }

  void draw_ui() {
    if (!show_window) {
      return;
    }

    auto& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && io.MouseClicked[1]) {
      clicked_pos_2d = Eigen::Vector2i(io.MousePos[0], io.MousePos[1]);
      clicked_depth = viewer->pick_depth(clicked_pos_2d);
      clicked_pos_3d = viewer->unproject(clicked_pos_2d, clicked_depth);
    }

    ImGui::Begin("picking", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("picked_pos_2d:%d %d", clicked_pos_2d[0], clicked_pos_2d[1]);
    ImGui::Text("picked_depth :%.3f", clicked_depth);
    ImGui::Text("picked_pos_3d:%.3f %.3f %.3f", clicked_pos_3d[0], clicked_pos_3d[1], clicked_pos_3d[2]);

    ImGui::End();
  }

  LightViewer* viewer;

  bool show_window;

  Eigen::Vector2i clicked_pos_2d;

  float clicked_depth;
  Eigen::Vector3f clicked_pos_3d;
};

/**
 * ViewerUI::ViewerUI
 */
LightViewer::ViewerUI::ViewerUI(guik::LightViewer* viewer) : viewer(viewer) {
  display_setting_window.reset(new DisplaySettingWindow(viewer));
  drawable_filter_window.reset(new DrawableFilterWindow(viewer));
  drawable_editor_window.reset(new DrawableEditorWindow(viewer));
  camera_setting_window.reset(new CameraSettingWindow(viewer));
  plot_setting_window.reset(new PlotSettingWindow(viewer));
  imgui_demo_windows.reset(new ImGuiDemoWindows(viewer));
  point_picking_window.reset(new PointPickingWindow(viewer));
}
/**
 * ViewerUI::ViewerUI
 */

LightViewer::ViewerUI::~ViewerUI() {}

/**
 * ViewerUI::draw_ui
 */
bool LightViewer::ViewerUI::draw_ui() {
  if (draw_main_menu_bar()) {
    return false;
  }

  display_setting_window->draw_ui();
  drawable_filter_window->draw_ui();
  drawable_editor_window->draw_ui();
  camera_setting_window->draw_ui();
  imgui_demo_windows->draw_ui();
  point_picking_window->draw_ui();

  return true;
}

/**
 * ViewerUI::draw_main_menu_bar
 */
bool LightViewer::ViewerUI::draw_main_menu_bar() {
  bool hide_menu = false;
  ImGui::BeginMainMenuBar();
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Hide menu")) {
      hide_menu = true;
    }

    if (ImGui::MenuItem("Quit viewer")) {
      viewer->close();
    }

    if (ImGui::MenuItem("Force kill")) {
      auto result = pfd::message("Force kill", "This will kill the program immediately. Are you sure? ", pfd::choice::ok_cancel, pfd::icon::warning).result();
      if (result == pfd::button::ok) {
        exit(1);
      }
    }
    ImGui::EndMenu();
  }

  static bool turn_table = false;
  if (ImGui::BeginMenu("Display")) {
    display_setting_window->menu_item();
    if (ImGui::MenuItem("Show Info Window")) {
      viewer->show_info_window();
    }
    if (ImGui::MenuItem("Show Sub Viewers")) {
      viewer->show_sub_viewers();
    }
    if (ImGui::MenuItem("Enable vsync")) {
      viewer->enable_vsync();
    }
    if (ImGui::MenuItem("Disable vsync")) {
      viewer->disable_vsync();
    }
    if (ImGui::MenuItem("Enable XY grid")) {
      viewer->set_draw_xy_grid(true);
    }
    if (ImGui::MenuItem("Disable XY grid")) {
      viewer->set_draw_xy_grid(false);
    }

    if (ImGui::MenuItem("Turn table", nullptr, &turn_table)) {
    }

    ImGui::EndMenu();
  }

  if (turn_table) {
    auto camera = viewer->get_camera_control();
    camera->mouse({0.0f, 0.0f}, 0, true);
    camera->drag({1.0f, 0.0f}, 0);
  }

  if (ImGui::BeginMenu("Drawable")) {
    drawable_filter_window->menu_item();
    drawable_editor_window->menu_item();
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Camera")) {
    camera_setting_window->menu_item();
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Plot")) {
    plot_setting_window->menu_item();
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Utility")) {
    imgui_demo_windows->menu_item();
    point_picking_window->menu_item();
    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();

  return hide_menu;
}

}  // namespace guik