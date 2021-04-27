#version 330

uniform sampler2D neighbor_counts_sampler;
uniform sampler2D radius_bounds_sampler;

uniform int k_neighbors;

in vec3 uvd;
in vec2 screen_pos;

out float finalized_radius;

void main() {
  vec4 num_neighbors = texture(neighbor_counts_sampler, uvd.xy);

  bvec4 found = lessThan(abs(num_neighbors - vec4(k_neighbors)), vec4(0.5));
  if(!found.x && !found.y && !found.z && !found.w) {
    discard;
  }

  vec2 radius_bounds = texture(radius_bounds_sampler, uvd.xy).xy;
  finalized_radius = radius_bounds.y;
  gl_FragDepth = 0.0;
}