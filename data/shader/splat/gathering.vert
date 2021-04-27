#version 330

uniform sampler2D position_sampler;
uniform sampler2D radius_bounds_sampler;
uniform sampler2D feedback_radius_sampler;

uniform vec2 screen_size;
uniform vec2 inv_screen_size;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;

in vec3 vert_position;

out vec3 p2_pos;

void main() {
  p2_pos = texture(position_sampler, vert_position.xy).xyz;
  float feedback_radius_world = texture(feedback_radius_sampler, vert_position.xy).x;

  if(feedback_radius_world < 1e-3) {
    gl_Position = vec4(0.0, 0.0, 10.0, 1.0);
    gl_PointSize = 1.0;
    return;
  }

  vec4 p2_pos_cam = view_matrix * vec4(p2_pos, 1.0);
  p2_pos_cam.xy = vec2(feedback_radius_world);

  vec4 p2_size_screen = projection_matrix * p2_pos_cam;
  vec2 feedback_radius_xy = abs(p2_size_screen.xy / p2_size_screen.w) * screen_size;
  float feedback_radius_screen = max(feedback_radius_xy.x, feedback_radius_xy.y);

  vec2 screen_pos = vert_position.xy * 2.0 - vec2(1.0);
  gl_Position = vec4(screen_pos, vert_position.z, 1.0);
  gl_PointSize = ceil(feedback_radius_screen) + 1;
  // gl_PointSize = min(feedback_radius, 64);
}