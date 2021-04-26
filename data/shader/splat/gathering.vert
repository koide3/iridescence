#version 330

uniform sampler2D position_sampler;
uniform sampler2D radius_bounds_sampler;
uniform sampler2D feedback_radius_sampler;

uniform vec2 inv_screen_size;

in vec3 vert_position;

out vec3 p2_pos;

void main() {
  p2_pos = texture(position_sampler, vert_position.xy).xyz;
  float feedback_radius = texture(feedback_radius_sampler, vert_position.xy).x;

  vec2 screen_pos = vert_position.xy * 2.0 - vec2(1.0);
  gl_Position = vec4(screen_pos, vert_position.z, 1.0);
  gl_PointSize = ceil(feedback_radius);
  // gl_PointSize = min(feedback_radius, 64);
}