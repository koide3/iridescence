#include <guik/model_control.hpp>

#include <vector>
#include <sstream>

#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Core>

#include <guik/viewer/light_viewer.hpp>

namespace guik {

ModelControl::ModelControl(const std::string& name, const Eigen::Matrix4f& init_model_matrix) : name(name), pose(init_model_matrix), gizmo_operation(ImGuizmo::OPERATION::ROTATE) {}

void ModelControl::draw_ui() {
  ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  draw_gizmo_ui();

  float value = 0.0f;
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PX", &value, 0.01f, 0.0f, 0.0f, "PX")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitX() * value);
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PY", &value, 0.01f, 0.0f, 0.0f, "PY")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitY() * value);
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PZ", &value, 0.01f, 0.0f, 0.0f, "PZ")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitZ() * value);
  }

  value = 0.0f;
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RX", &value, 0.01f, 0.0f, 0.0f, "RX")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitX());
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RY", &value, 0.01f, 0.0f, 0.0f, "RY")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitY());
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RZ", &value, 0.01f, 0.0f, 0.0f, "RZ")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitZ());
  }

  value = 0.0f;
  ImGui::SetNextItemWidth(100);
  if (ImGui::DragFloat("##SCALE", &value, 0.01f, 0.0f, 0.0f, "SCALE")) {
    pose = pose * Eigen::UniformScaling<float>(1.0f + value);
  }

  std::stringstream sst;
  sst << "trans:" << pose.translation().transpose() << "\n"
      << "quat :" << Eigen::Quaternionf(pose.rotation()).coeffs().transpose() << "\n"
      << "scale:" << pose.linear().colwise().norm() << std::endl;
  ImGui::Text("%s", sst.str().c_str());

  ImGui::End();
}

void ModelControl::draw_gizmo_ui() {
  int op = static_cast<int>(gizmo_operation);
  std::vector<const char*> operations = {"TRANSLATE", "ROTATE", "SCALE"};
  ImGui::Combo("Gizmo mode", &op, operations.data(), operations.size());
  gizmo_operation = static_cast<ImGuizmo::OPERATION>(op);
}

void ModelControl::draw_gizmo() {
  auto viewer = guik::LightViewer::instance();
  auto size = viewer->canvas_size();

  draw_gizmo(0, 0, size[0], size[1], viewer->view_matrix(), viewer->projection_matrix(), false);
}

void ModelControl::draw_gizmo(int win_x, int win_y, int win_w, int win_h, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, bool on_window) {
  ImGuizmo::Enable(true);
  if(on_window) {
    ImGuizmo::SetDrawlist();
  }
  ImGuizmo::SetRect(win_x, win_y, win_w, win_h);

  Eigen::Matrix4f model = pose.matrix();
  Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
  ImGuizmo::Manipulate(view.data(), projection.data(), static_cast<ImGuizmo::OPERATION>(gizmo_operation), ImGuizmo::MODE::LOCAL,
                       model.data(), delta.data());

  pose = Eigen::Affine3f(model);
}

bool ModelControl::is_guizmo_using() const {
  return ImGuizmo::IsUsing();
}

const std::string& ModelControl::model_name() const { return name; }

Eigen::Matrix4f ModelControl::model_matrix() const { return pose.matrix(); }

}  // namespace guik
