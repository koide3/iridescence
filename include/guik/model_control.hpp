#ifndef GUIK_MODEL_CONTROL_HPP
#define GUIK_MODEL_CONTROL_HPP

#include <sstream>

#include <imgui.h>
#include <Eigen/Dense>

namespace guik {

class ModelControl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ModelControl(const std::string& name) 
    : name(name),
    pose(Eigen::Isometry3f::Identity())
    {}

    void draw_ui() {
        ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        float value = 0.0f;
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##PX", &value, 0.01f, 0.0f, 0.0f, "PX")) {
            pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitX() * value);
        }

        value = 0.0f;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##PY", &value, 0.01f, 0.0f, 0.0f, "PY")) {
            pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitY() * value);
        }

        value = 0.0f;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##PZ", &value, 0.01f, 0.0f, 0.0f, "PZ")) {
            pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitZ() * value);
        }

        value = 0.0f;
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##RX", &value, 0.01f, 0.0f, 0.0f, "RX")) {
            pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitX());
        }

        value = 0.0f;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##RY", &value, 0.01f, 0.0f, 0.0f, "RY")) {
            pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitY());
        }

        value = 0.0f;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        if(ImGui::DragFloat("##RZ", &value, 0.01f, 0.0f, 0.0f, "RZ")) {
            pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitZ());
        }

        value = 0.0f;
        ImGui::SetNextItemWidth(100);
        if(ImGui::DragFloat("##SCALE", &value, 0.01f, 0.0f, 0.0f, "SCALE")) {
            pose = pose * Eigen::UniformScaling<float>(1.0f + value);
        }


        std::stringstream sst;
        sst << "trans:" << pose.translation().transpose() << "\n"
            << "quat :" << Eigen::Quaternionf(pose.rotation()).coeffs().transpose() << "\n"
            << "scale:" << pose.linear().colwise().norm() << std::endl;
        ImGui::Text(sst.str().c_str());

        ImGui::End();
    }

    Eigen::Matrix4f model_matrix() const {
        return pose.matrix();
    }

private:
    std::string name;
    Eigen::Affine3f pose;
};

}

#endif