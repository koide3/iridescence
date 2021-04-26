#version 330

uniform sampler2D position_sampler;
uniform sampler2D radius_sampler;

uniform vec2 screen_size;
uniform vec2 inv_screen_size;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;

in vec3 vert_position;

out vec3 p1_pos;
out float p1_radius;
out float p1_radius_screen;

void main() {
  p1_pos = texture(position_sampler, vert_position.xy).xyz;
  p1_radius = texture(radius_sampler, vert_position.xy).y;

  vec4 p1_pos_cam = view_matrix * vec4(p1_pos, 1.0);
  p1_pos_cam.xy = vec2(p1_radius);

  vec4 p1_size_screen = projection_matrix * p1_pos_cam;
  vec2 radius_xy = abs(p1_size_screen.xy / p1_size_screen.w) * screen_size;
  p1_radius_screen = max(radius_xy.x, radius_xy.y);

  vec2 screen_pos = vert_position.xy * 2.0 - vec2(1.0);
  gl_Position = vec4(screen_pos, vert_position.z, 1.0);
  gl_PointSize = ceil(p1_radius_screen) + 1;
  // gl_PointSize = min(p1_radius_screen * 1.5, 128);
}