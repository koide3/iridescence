#include <guik/camera/sensor_view_camera_control.hpp>

#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace guik {

SensorViewCameraControl::SensorViewCameraControl(const Eigen::Isometry3f& T_sensor_camera, double smoothing_factor_trans, double smoothing_factor_rot)
: T_sensor_camera(T_sensor_camera),
  T_world_sensor(Eigen::Isometry3f::Identity()),
  T_world_sensor_smoothed(Eigen::Isometry3f::Identity()),
  smoothing_factor_trans(std::clamp(smoothing_factor_trans, 0.0, 1.0)),
  smoothing_factor_rot(std::clamp(smoothing_factor_rot, 0.0, 1.0)),
  first_pose(true),
  z_axis_alignment(true),
  left_button_down(false),
  theta(0.0),
  phi(0.0),
  distance_scale(1.0) {
  init_distance = T_sensor_camera.translation().norm();
  if (init_distance < 1e-6) {
    init_distance = 1.0;
  }
}

SensorViewCameraControl::~SensorViewCameraControl() {}

void SensorViewCameraControl::set_smoothing_factor(double trans, double rot) {
  smoothing_factor_trans = std::clamp(trans, 0.0, 1.0);
  smoothing_factor_rot = std::clamp(rot, 0.0, 1.0);
}

void SensorViewCameraControl::set_smoothing_factor_trans(double factor) {
  smoothing_factor_trans = std::clamp(factor, 0.0, 1.0);
}

void SensorViewCameraControl::set_smoothing_factor_rot(double factor) {
  smoothing_factor_rot = std::clamp(factor, 0.0, 1.0);
}

void SensorViewCameraControl::set_z_axis_alignment(bool enabled) {
  z_axis_alignment = enabled;
}

void SensorViewCameraControl::set_sensor_pose(const Eigen::Isometry3f& T_world_sensor) {
  this->T_world_sensor = T_world_sensor;

  if (first_pose) {
    T_world_sensor_smoothed = T_world_sensor;
    first_pose = false;
    return;
  }

  // Interpolation weights: 1 means snap to new pose, ~0 means very slow follow.
  const float alpha_trans = static_cast<float>(1.0 - smoothing_factor_trans);
  const float alpha_rot = static_cast<float>(1.0 - smoothing_factor_rot);

  // Lerp translation
  T_world_sensor_smoothed.translation() = T_world_sensor_smoothed.translation() * (1.0f - alpha_trans) + T_world_sensor.translation() * alpha_trans;

  // Slerp rotation
  Eigen::Quaternionf q_current(T_world_sensor_smoothed.rotation());
  Eigen::Quaternionf q_target(T_world_sensor.rotation());
  T_world_sensor_smoothed.linear() = q_current.slerp(alpha_rot, q_target).toRotationMatrix();
}

void SensorViewCameraControl::set_sensor_camera_transform(const Eigen::Isometry3f& T_sensor_camera) {
  this->T_sensor_camera = T_sensor_camera;
}

void SensorViewCameraControl::mouse(const Eigen::Vector2f& p, int button, bool down) {
  if (button == 0) {
    left_button_down = down;
  }
  drag_last_pos = p;
}

void SensorViewCameraControl::drag(const Eigen::Vector2f& p, int button) {
  Eigen::Vector2f rel = p - drag_last_pos;
  if (left_button_down) {
    theta -= rel[0] * 0.01;
    phi += rel[1] * 0.01;
    phi = std::clamp(phi, -M_PI_2 + 0.01, M_PI_2 - 0.01);
  }
  drag_last_pos = p;
}

void SensorViewCameraControl::scroll(const Eigen::Vector2f& rel) {
  if (rel[0] > 0) {
    distance_scale *= 0.8;
  } else if (rel[0] < 0) {
    distance_scale *= 1.2;
  }
  distance_scale = std::max(0.01, distance_scale);
}

Eigen::Vector2f SensorViewCameraControl::depth_range() const {
  double dist = init_distance * distance_scale;
  return Eigen::Vector2f(dist / 10.0f, dist * 100.0f);
}

Eigen::Matrix4f SensorViewCameraControl::view_matrix() const {
  // Sensor position in world frame (the point the camera always looks at)
  Eigen::Vector3f sensor_pos = T_world_sensor_smoothed.translation();

  double dist = init_distance * distance_scale;

  if (z_axis_alignment) {
    // Z-axis aligned mode: orbit camera with world-Z as up direction
    Eigen::Vector3f init_offset_world = T_world_sensor_smoothed.rotation() * T_sensor_camera.translation();
    Eigen::Vector3f base_dir = init_offset_world.normalized();

    double base_theta = std::atan2(static_cast<double>(base_dir.y()), static_cast<double>(base_dir.x()));
    double base_phi = std::asin(std::clamp(static_cast<double>(base_dir.z()), -1.0, 1.0));

    double total_theta = base_theta + theta;
    double total_phi = std::clamp(base_phi + phi, -M_PI_2 + 0.01, M_PI_2 - 0.01);

    Eigen::Vector3f offset(
      static_cast<float>(dist * std::cos(total_phi) * std::cos(total_theta)),
      static_cast<float>(dist * std::cos(total_phi) * std::sin(total_theta)),
      static_cast<float>(dist * std::sin(total_phi)));

    Eigen::Vector3f eye = sensor_pos + offset;
    Eigen::Vector3f up = Eigen::Vector3f::UnitZ();

    glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(sensor_pos[0], sensor_pos[1], sensor_pos[2]), glm::vec3(up[0], up[1], up[2]));
    return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
  } else {
    // Non-aligned mode: camera pose is purely T_world_sensor * T_sensor_camera, with user orbit in sensor frame
    Eigen::Isometry3f T_sensor_camera = Eigen::Isometry3f::Identity();
    Eigen::Vector3f cam_offset = Eigen::AngleAxisf(static_cast<float>(theta), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(static_cast<float>(phi), Eigen::Vector3f::UnitY()) *
                                 (T_sensor_camera.translation().normalized() * static_cast<float>(dist));
    T_sensor_camera.translation() = cam_offset;

    // Camera looks from eye toward sensor origin, with sensor-z as up
    Eigen::Isometry3f T_world_camera = T_world_sensor_smoothed * T_sensor_camera;
    Eigen::Vector3f eye = T_world_camera.translation();
    Eigen::Vector3f up = T_world_sensor_smoothed.rotation() * Eigen::Vector3f::UnitZ();

    glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(sensor_pos[0], sensor_pos[1], sensor_pos[2]), glm::vec3(up[0], up[1], up[2]));
    return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat)).eval();
  }
}

}  // namespace guik
