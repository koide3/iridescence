#version 330

// layout(early_fragment_tests) in;

uniform sampler2D position_sampler;
uniform sampler2D radius_bounds_sampler;
uniform sampler2D feedback_radius_sampler;

uniform vec2 inv_screen_size;

in vec3 p2_pos;

out vec4 neighbor_counts;

void main() {
  vec2 p1_texcoord = gl_FragCoord.xy * inv_screen_size;
  vec3 p1_pos = texture(position_sampler, p1_texcoord).xyz;
  vec2 p1_radius_bounds = texture(radius_bounds_sampler, p1_texcoord).xy;

  float dist = length(p1_pos - p2_pos);
  if(dist > p1_radius_bounds.y) {
    discard;
  }

  float l = p1_radius_bounds.x;
  float h = p1_radius_bounds.y;
  float w = (h - l) / 4;

  vec4 check_radii = vec4(l + w, l + 2 * w, l + 3 * w, h);
  neighbor_counts = vec4(lessThan(vec4(dist), check_radii));
}