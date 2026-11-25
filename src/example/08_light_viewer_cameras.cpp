#include <iostream>
#include <glk/primitives/primitives.hpp>
#include <guik/camera/fps_camera_control.hpp>
#include <guik/camera/sensor_view_camera_control.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::viewer();

  // Static scene: a grid of cubes with different heights
  for (int x = -4; x <= 4; x++) {
    for (int y = -4; y <= 4; y++) {
      const float height = 1.0f + 0.5f * (std::abs(x) + std::abs(y));
      const Eigen::Affine3f transform = Eigen::Translation3f(x * 4.0f, y * 4.0f, height / 2.0f) * Eigen::Scaling(1.0f, 1.0f, height);
      viewer->update_cube("cube_" + std::to_string(x) + "_" + std::to_string(y), guik::Rainbow(transform));
    }
  }

  // Camera control types
  const std::vector<const char*> camera_modes = {"Orbit (XY)", "Orbit (XZ)", "TopDown", "ArcBall", "FPS", "SensorView"};
  const std::vector<const char*> camera_hints = {
    "Left drag: rotate / Middle drag: move / Scroll: zoom\nZ-up orbit camera suitable for XYZ = front-left-up systems (e.g., LiDAR).",
    "Left drag: rotate / Middle drag: move / Scroll: zoom\nOrbit camera suitable for XYZ = right-down-forward systems (e.g., cameras).",
    "Left drag: rotate / Middle drag: move / Scroll: zoom\nTop-down view suitable for 2D maps.",
    "Left drag: rotate / Middle drag: move / Scroll: zoom\nArcball camera that can take an arbitrary pose.",
    "W/A/S/D: move / R/F: up and down / E/Q: yaw\nLeft drag: yaw and pitch / Scroll: FOV",
    "Left drag: rotate around the sensor / Scroll: distance\nThe camera follows the sensor pose with smoothing."};
  int camera_mode = 0;

  // Projection types
  bool orthographic = false;
  float fovy_deg = 30.0f;
  float ortho_width = 50.0f;
  const auto set_projection_control = [&] {
    if (orthographic) {
      viewer->use_orthographic_projection_control(ortho_width);
    } else {
      viewer->use_perspective_projection_control(fovy_deg);
    }
  };

  // Only SensorViewCameraControl needs to be updated with the sensor pose every frame
  std::shared_ptr<guik::SensorViewCameraControl> sensor_camera;

  const auto set_camera_control = [&] {
    sensor_camera = nullptr;
    switch (camera_mode) {
      case 0:  // (distance, theta, phi)
        viewer->use_orbit_camera_control(50.0, 0.0, -60.0 * M_PI / 180.0);
        break;
      case 1:
        viewer->use_orbit_camera_control_xz(50.0, 0.0, 0.0);
        break;
      case 2:  // (distance, theta)
        viewer->use_topdown_camera_control(50.0, 0.0);
        break;
      case 3:
        viewer->use_arcball_camera_control(50.0, 0.0, -60.0 * M_PI / 180.0);
        break;
      case 4: {
        auto fps_camera = viewer->use_fps_camera_control(60.0);
        fps_camera->set_pose(Eigen::Vector3f(-30.0f, 0.0f, 10.0f), 0.0, -15.0);  // (position, yaw_deg, pitch_deg)
        fps_camera->set_translation_speed(0.2);
        break;
      }
      case 5:  // T_sensor_camera: the camera is placed behind and above the sensor
        sensor_camera = viewer->use_sensor_view_camera_control(Eigen::Translation3f(-10.0f, 0.0f, 4.0f) * Eigen::Isometry3f::Identity());
        break;
    }

    // The FPS camera control also works as the projection control (it replaces the current one).
    // The projection control must be set again when switching to another camera control.
    if (camera_mode != 4) {
      set_projection_control();
    }
  };
  set_camera_control();

  bool follow_sensor = false;
  const std::string settings_path = "/tmp/iridescence_camera_settings.txt";
  Eigen::Isometry3f T_world_sensor = Eigen::Isometry3f::Identity();

  viewer->register_ui_callback("camera_ui", [&] {
    ImGui::Begin("cameras", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Camera control (view matrix)
    if (ImGui::Combo("camera control", &camera_mode, camera_modes.data(), camera_modes.size())) {
      set_camera_control();
    }
    ImGui::TextUnformatted(camera_hints[camera_mode]);

    // Projection control (projection matrix)
    ImGui::Separator();
    if (camera_mode == 4) {
      ImGui::TextUnformatted("The FPS camera controls the projection by itself.");
    } else {
      bool projection_changed = ImGui::Checkbox("orthographic", &orthographic);
      if (orthographic) {
        projection_changed |= ImGui::DragFloat("width", &ortho_width, 0.1f, 1.0f, 500.0f);
      } else {
        projection_changed |= ImGui::DragFloat("fovy [deg]", &fovy_deg, 0.1f, 5.0f, 120.0f);
      }
      if (projection_changed) {
        set_projection_control();
      }
    }

    // Move the camera programmatically
    ImGui::Separator();
    if (ImGui::Button("lookat sensor")) {
      viewer->lookat(T_world_sensor.translation());  // Move the camera center to the point
    }
    ImGui::SameLine();
    if (ImGui::Button("lookat origin")) {
      viewer->lookat(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    }
    ImGui::SameLine();
    if (ImGui::Button("reset center")) {
      viewer->reset_center();
    }
    ImGui::Checkbox("follow sensor (lookat every frame)", &follow_sensor);

    // Save and load the camera and projection settings
    ImGui::Separator();
    if (ImGui::Button("save settings")) {
      viewer->append_text(viewer->save_camera_settings(settings_path) ? "saved to " + settings_path : "failed to save camera settings");
    }
    ImGui::SameLine();
    if (ImGui::Button("load settings")) {
      // Loading replaces the camera and projection controls with the saved ones
      // Note: Only the orbit, top-down, and arcball camera controls (with the perspective/orthographic projection) can be saved and restored
      if (viewer->load_camera_settings(settings_path)) {
        sensor_camera = nullptr;
        viewer->append_text("loaded from " + settings_path);
      } else {
        viewer->append_text("failed to load camera settings");
      }
    }

    // The view matrix can be obtained from the camera control
    ImGui::Separator();
    const Eigen::Isometry3f T_world_camera(viewer->view_matrix().inverse());
    const Eigen::Vector3f pos = T_world_camera.translation();
    ImGui::Text("camera position: %.2f %.2f %.2f", pos.x(), pos.y(), pos.z());

    ImGui::End();
  });

  double t = 0.0;
  while (viewer->spin_once()) {
    // Moving "sensor" running on a figure-eight path
    t += ImGui::GetIO().DeltaTime * 0.3;
    const Eigen::Vector3f pos(22.0 * std::sin(t), 14.0 * std::sin(2.0 * t), 6.0);
    const Eigen::Vector3f vel(22.0 * std::cos(t), 28.0 * std::cos(2.0 * t), 0.0);
    T_world_sensor = Eigen::Translation3f(pos) * Eigen::AngleAxisf(std::atan2(vel.y(), vel.x()), Eigen::Vector3f::UnitZ());

    viewer->update_coord("sensor", guik::VertexColor(T_world_sensor * Eigen::UniformScaling<float>(2.0f)));
    viewer->update_wire_sphere("sensor_sphere", guik::FlatOrange(T_world_sensor));

    if (sensor_camera) {
      // SensorViewCameraControl follows the given sensor pose
      sensor_camera->set_sensor_pose(T_world_sensor);
    } else if (follow_sensor) {
      viewer->lookat(T_world_sensor.translation());
    }
  }

  return 0;
}
