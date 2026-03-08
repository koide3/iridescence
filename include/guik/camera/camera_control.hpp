#ifndef GLK_CAMERA_CONTROL_HPP
#define GLK_CAMERA_CONTROL_HPP

#include <memory>
#include <iostream>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

/// @brief Base class for camera control. This class defines the interface for controlling the camera in the viewer.
///        It provides virtual methods for handling mouse input, updating the camera state, and getting the view matrix.
///        Derived classes can implement specific camera control schemes (e.g., orbit, FPV) by overriding these methods.
/// @note  Many of default camera control methods have an interface that combines a programmatic API (e.g., lookat()) and
///        user input (e.g., mouse dragging). For example, the lookat() method can be used to programmatically set the
///        camera to look at a specific point in the world, while the mouse dragging can be used to shift the camera's
///        center of focus. The reset_center() method can be used to reset any such shifts.
class CameraControl {
public:
  virtual ~CameraControl() {}

  /// @brief Reset the shift of the camera.
  virtual void reset_center() {}

  /// @brief Move the camera to look at the given point. The point is expected to be in world coordinates.
  /// @param pt The point to look at, in world coordinates.
  virtual void lookat(const Eigen::Vector3f& pt) {}

  /// @brief Mouse event callback. This method is called when a mouse button is pressed or released.
  /// @param p       The current mouse position in screen coordinates (pixels).
  /// @param button  The mouse button index (0: left, 1: right, 2: middle).
  /// @param down    True if the button is pressed, false if released.
  virtual void mouse(const Eigen::Vector2f& p, int button, bool down) {}

  /// @brief Mouse drag event callback. This method is called when the mouse is moved while a button is pressed.
  /// @param p       The current mouse position in screen coordinates (pixels).
  /// @param button  The mouse button index (0: left, 1: right, 2: middle).
  virtual void drag(const Eigen::Vector2f& p, int button) {}

  /// @brief Scroll event callback. This method is called when the mouse wheel is scrolled.
  /// @param rel The relative scroll amount.
  virtual void scroll(const Eigen::Vector2f& rel) {}

  /// @brief Update the camera state. This method can be called every frame to update the camera based on user input or animations.
  virtual void update() {}

  /// @brief Keyboard page up/down event callback. This method is called while the user is pressing the page up/down keys.
  /// @param p The amount of movement. Positive for page up, negative for page down.
  virtual void updown(double p) {}

  /// @brief Keyboard arrow key event callback. This method is called while the user is pressing the arrow keys.
  /// @param p The amount of movement in the x and y directions.
  virtual void arrow(const Eigen::Vector2f& p) {}

  /// @brief Get the depth range of the camera. This method can be overridden by derived classes to specify a custom depth range for rendering.
  /// @return The near and far depth values as a 2D vector (x: near, y: far).
  virtual Eigen::Vector2f depth_range() const { return Eigen::Vector2f(0.0f, 1.0f); }

  /// @brief Get the view matrix of the camera. This method must be implemented by derived classes to return the current view transformation matrix.
  /// @return The 4x4 view matrix as an Eigen::Matrix4f.
  virtual Eigen::Matrix4f view_matrix() const = 0;

  // io
  virtual std::string name() const { return "CameraControl"; }
  virtual void load(std::istream& ist) {}
  virtual void save(std::ostream& ost) const {}
};

std::istream& operator>>(std::istream& ist, CameraControl& cam);
std::ostream& operator<<(std::ostream& ost, const CameraControl& cam);

}  // namespace guik

#endif