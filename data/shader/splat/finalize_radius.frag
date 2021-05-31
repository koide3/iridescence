#version 330

uniform sampler2D neighbor_counts_sampler;
uniform sampler2D radius_bounds_sampler;

uniform int k_neighbors;

in vec3 uvd;
in vec2 screen_pos;

out vec2 finalized_radius;

void main() {
  vec4 num_neighbors = texture(neighbor_counts_sampler, uvd.xy);

  bvec4 found = lessThan(abs(num_neighbors - vec4(k_neighbors)), vec4(0.5));
  if(!any(found)) {
    discard;
  }

  vec2 radius_bounds = texture(radius_bounds_sampler, uvd.xy).xy;

  float l = radius_bounds.x;
  float h = radius_bounds.y;
  float w = (h - l) / 4;

  if(found.x) {
    finalized_radius = vec2(l + w);
  } else if(found.y) {
    finalized_radius = vec2(l + 2 * w);
  } else if(found.z) {
    finalized_radius = vec2(l + 3 * w);
  } else {
    finalized_radius = vec2(h);
  }

  gl_FragDepth = 0.0;
}