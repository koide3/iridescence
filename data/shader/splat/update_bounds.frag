#version 330

const float max_radius = 0.5;

uniform sampler2D neighbor_counts_sampler;
uniform sampler2D radius_bounds_sampler;

uniform int k_neighbors;

in vec3 uvd;
in vec2 screen_pos;

out vec2 updated_radius_bounds;

void main() {
  ivec4 num_neighbors = ivec4(texture(neighbor_counts_sampler, uvd.xy));
  vec2 current_radius_bounds = texture(radius_bounds_sampler, uvd.xy).xy;

  if(num_neighbors.w == k_neighbors) {
    updated_radius_bounds = current_radius_bounds;
    return;
  }

  if(num_neighbors.w < k_neighbors) {
    float alpha = sqrt(float(k_neighbors) / max(1, num_neighbors.w));
    // alpha = clamp(alpha, 1.0, 10.0);
    updated_radius_bounds.x = current_radius_bounds.x;
    updated_radius_bounds.y = min(alpha * current_radius_bounds.y, max_radius);
    return;
  }

  float l = current_radius_bounds.x;
  float h = current_radius_bounds.y;
  float w = (h - l) / 4;
  vec4 check_radii = vec4(l + w, l + 2 * w, l + 3 * w, h);

  if(num_neighbors.x >= k_neighbors) {
    updated_radius_bounds.x = current_radius_bounds.x;
    updated_radius_bounds.y = check_radii.x;
    return;
  }

  if(num_neighbors.y >= k_neighbors) {
    updated_radius_bounds = check_radii.xy;
    return;
  }

  if(num_neighbors.z >= k_neighbors) {
    updated_radius_bounds = check_radii.yz;
    return;
  }

  updated_radius_bounds = check_radii.zw;
}