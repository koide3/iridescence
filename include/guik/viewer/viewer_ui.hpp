#ifndef LIGHT_VIEWER_UI_HPP
#define LIGHT_VIEWER_UI_HPP

#include <regex>
#include <imgui.h>
#include <guik/model_control.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace guik {

class LightViewer::ViewerUI {
public:
  ViewerUI(guik::LightViewer* viewer);
  ~ViewerUI();

  bool draw_ui();

private:
  bool draw_main_menu_bar();

private:
  guik::LightViewer* viewer;

  class DisplaySettingWindow;

  class DrawableFilterWindow;
  class DrawableEditorWindow;
  class CameraSettingWindow;
  class PlotSettingWindow;
  class ImGuiDemoWindows;
  class PointPickingWindow;

  std::unique_ptr<DisplaySettingWindow> display_setting_window;

  std::unique_ptr<DrawableFilterWindow> drawable_filter_window;
  std::unique_ptr<DrawableEditorWindow> drawable_editor_window;
  std::unique_ptr<CameraSettingWindow> camera_setting_window;
  std::unique_ptr<PlotSettingWindow> plot_setting_window;
  std::unique_ptr<ImGuiDemoWindows> imgui_demo_windows;
  std::unique_ptr<PointPickingWindow> point_picking_window;
};
}  // namespace guik

#endif