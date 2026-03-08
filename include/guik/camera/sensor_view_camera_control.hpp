#ifndef GUIK_SENSOR_VIEW_CAMERA_CONTROL_HPP
#define GUIK_SENSOR_VIEW_CAMERA_CONTROL_HPP

#include <guik/camera/camera_control.hpp>

namespace guik {

/// @brief Camera control that orbits around a tracked sensor pose. The camera always looks at the sensor position, with user-controlled azimuth/elevation and distance scaling.
class SensorViewCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @param T_sensor_camera         Transform from sensor frame to camera frame.
  /// @param smoothing_factor_trans  Smoothing factor for translation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
  /// @param smoothing_factor_rot    Smoothing factor for rotation in [0, 1]. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
  SensorViewCameraControl(
    const Eigen::Isometry3f& T_sensor_camera = Eigen::Translation3f(-5.0f, 0.0f, 0.1f) * Eigen::Isometry3f::Identity(),
    double smoothing_factor_trans = 0.75,
    double smoothing_factor_rot = 0.9);
  virtual ~SensorViewCameraControl() override;

  /// @brief Set both smoothing factors. 0 = no smoothing (instant snap), 1 = maximum smoothing (very slow follow).
  void set_smoothing_factor(double trans, double rot);

  /// @brief Set the translation smoothing factor.
  void set_smoothing_factor_trans(double factor);

  /// @brief Set the rotation smoothing factor.
  void set_smoothing_factor_rot(double factor);

  /// @brief Enable or disable z-axis alignment. When enabled (default), the camera up direction is locked to the world z-axis.
  void set_z_axis_alignment(bool enabled);

  /// @brief Set the raw sensor pose (T_world_sensor).
  void set_sensor_pose(const Eigen::Isometry3f& T_world_sensor);

  /// @brief Set the transform from sensor frame to camera frame (T_sensor_camera).
  void set_sensor_camera_transform(const Eigen::Isometry3f& T_sensor_camera);

  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) override;
  virtual void drag(const Eigen::Vector2f& p, int button) override;
  virtual void scroll(const Eigen::Vector2f& rel) override;

  virtual Eigen::Vector2f depth_range() const override;
  virtual Eigen::Matrix4f view_matrix() const override;

  virtual std::string name() const override { return "SensorViewCameraControl"; }

private:
  Eigen::Isometry3f T_sensor_camera;
  Eigen::Isometry3f T_world_sensor;           ///< Raw (unsmoothed) sensor pose
  Eigen::Isometry3f T_world_sensor_smoothed;  ///< Smoothed sensor pose used for rendering
  double smoothing_factor_trans;              ///< Translation smoothing factor in [0, 1]
  double smoothing_factor_rot;                ///< Rotation smoothing factor in [0, 1]
  bool first_pose;                            ///< True until the first pose is received
  bool z_axis_alignment;                      ///< If true, camera up is locked to world Z; if false, follows sensor frame

  double init_distance;
  double distance_scale;
  double theta;  ///< User azimuth adjustment (radians, around world Z)
  double phi;    ///< User elevation adjustment (radians, from XY plane)

  bool left_button_down;
  Eigen::Vector2f drag_last_pos;
};

}  // namespace guik

#endif
