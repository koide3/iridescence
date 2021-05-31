#version 330

// layout(early_fragment_tests) in;

uniform sampler2D position_sampler;
uniform sampler2D radius_bounds_sampler;
uniform sampler2D feedback_radius_sampler;

uniform vec2 inv_screen_size;

in vec3 p2_pos;

out float num_points;
out vec3 sum_points;
out vec3 sum_cross1;
out vec3 sum_cross2;

void main() {
  vec2 p1_texcoord = gl_FragCoord.xy * inv_screen_size;
  vec3 p1_pos = texture(position_sampler, p1_texcoord).xyz;
  float p1_radius = texture(radius_bounds_sampler, p1_texcoord).y;

  float dist = length(p1_pos - p2_pos);
  if(dist > p1_radius) {
    discard;
  }

  vec3 p = p2_pos;

  num_points = 1;
  sum_points = p;
  sum_cross1 = vec3(p.x * p.x, p.x * p.y, p.x * p.z);
  sum_cross2 = vec3(p.y * p.y, p.y * p.z, p.z * p.z);
}