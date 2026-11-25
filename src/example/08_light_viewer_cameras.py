#!/usr/bin/python3
import numpy
from pyridescence import *


def translation(x, y, z):
  T = numpy.identity(4, dtype=numpy.float32)
  T[:3, 3] = [x, y, z]
  return T


viewer = guik.viewer()

# Static scene: a grid of cubes with different heights
for x in range(-4, 5):
  for y in range(-4, 5):
    height = 1.0 + 0.5 * (abs(x) + abs(y))
    transform = translation(x * 4.0, y * 4.0, height / 2.0) @ numpy.diag([1.0, 1.0, height, 1.0]).astype(numpy.float32)
    viewer.update_cube(f'cube_{x}_{y}', guik.Rainbow(transform))

# Camera control types
camera_modes = ['Orbit (XY)', 'Orbit (XZ)', 'TopDown', 'ArcBall', 'FPS', 'SensorView']
camera_hints = [
  'Left drag: rotate / Middle drag: move / Scroll: zoom\nZ-up orbit camera suitable for XYZ = front-left-up systems (e.g., LiDAR).',
  'Left drag: rotate / Middle drag: move / Scroll: zoom\nOrbit camera suitable for XYZ = right-down-forward systems (e.g., cameras).',
  'Left drag: rotate / Middle drag: move / Scroll: zoom\nTop-down view suitable for 2D maps.',
  'Left drag: rotate / Middle drag: move / Scroll: zoom\nArcball camera that can take an arbitrary pose.',
  'W/A/S/D: move / R/F: up and down / E/Q: yaw\nLeft drag: yaw and pitch / Scroll: FOV',
  'Left drag: rotate around the sensor / Scroll: distance\nThe camera follows the sensor pose with smoothing.',
]
camera_mode = 0

# Projection types
orthographic = False
fovy_deg = 30.0
ortho_width = 50.0


def set_projection_control():
  if orthographic:
    viewer.use_orthographic_projection_control(ortho_width)
  else:
    viewer.use_perspective_projection_control(fovy_deg)


# Only SensorViewCameraControl needs to be updated with the sensor pose every frame
sensor_camera = None


def set_camera_control():
  global sensor_camera
  sensor_camera = None
  if camera_mode == 0:  # (distance, theta, phi)
    viewer.use_orbit_camera_control(50.0, 0.0, numpy.deg2rad(-60.0))
  elif camera_mode == 1:
    viewer.use_orbit_camera_control_xz(50.0, 0.0, 0.0)
  elif camera_mode == 2:  # (distance, theta)
    viewer.use_topdown_camera_control(50.0, 0.0)
  elif camera_mode == 3:
    viewer.use_arcball_camera_control(50.0, 0.0, numpy.deg2rad(-60.0))
  elif camera_mode == 4:
    fps_camera = viewer.use_fps_camera_control(60.0)
    fps_camera.set_pose([-30.0, 0.0, 10.0], 0.0, -15.0)  # (position, yaw_deg, pitch_deg)
    fps_camera.set_translation_speed(0.2)
  elif camera_mode == 5:  # T_sensor_camera: the camera is placed behind and above the sensor
    sensor_camera = viewer.use_sensor_view_camera_control(translation(-10.0, 0.0, 4.0))

  # The FPS camera control also works as the projection control (it replaces the current one).
  # The projection control must be set again when switching to another camera control.
  if camera_mode != 4:
    set_projection_control()


set_camera_control()

follow_sensor = False
settings_path = '/tmp/iridescence_camera_settings.txt'
T_world_sensor = numpy.identity(4, dtype=numpy.float32)


def camera_ui():
  global camera_mode, orthographic, fovy_deg, ortho_width, follow_sensor, sensor_camera
  imgui.begin('cameras', True, imgui.WindowFlags_AlwaysAutoResize)

  # Camera control (view matrix)
  changed, camera_mode = imgui.combo('camera control', camera_mode, camera_modes)
  if changed:
    set_camera_control()
  imgui.text(camera_hints[camera_mode])

  # Projection control (projection matrix)
  imgui.separator()
  if camera_mode == 4:
    imgui.text('The FPS camera controls the projection by itself.')
  else:
    projection_changed, orthographic = imgui.checkbox('orthographic', orthographic)
    if orthographic:
      changed, ortho_width = imgui.drag_float('width', ortho_width, 0.1, 1.0, 500.0)
    else:
      changed, fovy_deg = imgui.drag_float('fovy [deg]', fovy_deg, 0.1, 5.0, 120.0)
    if projection_changed or changed:
      set_projection_control()

  # Move the camera programmatically
  imgui.separator()
  if imgui.button('lookat sensor'):
    viewer.lookat(T_world_sensor[:3, 3])  # Move the camera center to the point
  imgui.same_line()
  if imgui.button('lookat origin'):
    viewer.lookat([0.0, 0.0, 0.0])
  imgui.same_line()
  if imgui.button('reset center'):
    viewer.reset_center()
  _, follow_sensor = imgui.checkbox('follow sensor (lookat every frame)', follow_sensor)

  # Save and load the camera and projection settings
  imgui.separator()
  if imgui.button('save settings'):
    viewer.append_text(f'saved to {settings_path}' if viewer.save_camera_settings(settings_path) else 'failed to save camera settings')
  imgui.same_line()
  if imgui.button('load settings'):
    # Loading replaces the camera and projection controls with the saved ones
    # Note: Only the orbit, top-down, and arcball camera controls (with the perspective/orthographic projection) can be saved and restored
    if viewer.load_camera_settings(settings_path):
      sensor_camera = None
      viewer.append_text(f'loaded from {settings_path}')
    else:
      viewer.append_text('failed to load camera settings')

  # The view matrix can be obtained from the camera control
  imgui.separator()
  T_world_camera = numpy.linalg.inv(viewer.get_camera_control().view_matrix())
  imgui.text('camera position: {:.2f} {:.2f} {:.2f}'.format(*T_world_camera[:3, 3]))

  imgui.end()


viewer.register_ui_callback('camera_ui', camera_ui)

t = 0.0
while viewer.spin_once():
  # Moving "sensor" running on a figure-eight path
  t += imgui.get_io().delta_time * 0.3
  pos = numpy.array([22.0 * numpy.sin(t), 14.0 * numpy.sin(2.0 * t), 6.0])
  vel = numpy.array([22.0 * numpy.cos(t), 28.0 * numpy.cos(2.0 * t), 0.0])
  yaw = numpy.arctan2(vel[1], vel[0])

  T_world_sensor = numpy.identity(4, dtype=numpy.float32)
  T_world_sensor[:2, :2] = [[numpy.cos(yaw), -numpy.sin(yaw)], [numpy.sin(yaw), numpy.cos(yaw)]]
  T_world_sensor[:3, 3] = pos

  viewer.update_coord('sensor', guik.VertexColor(T_world_sensor).scale(2.0))
  viewer.update_wire_sphere('sensor_sphere', guik.FlatOrange(T_world_sensor))

  if sensor_camera is not None:
    # SensorViewCameraControl follows the given sensor pose
    sensor_camera.set_sensor_pose(T_world_sensor)
  elif follow_sensor:
    viewer.lookat(T_world_sensor[:3, 3])
