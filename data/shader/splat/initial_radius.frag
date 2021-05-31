#version 330

const float PI = 3.1415926535;
const float init_radius_scale = 2.0;
const float min_init_radius = 0.2;
const float max_init_radius = 2.0;

uniform sampler2D depth_sampler;
uniform sampler2D num_points_grid_sampler;

uniform int grid_area;
uniform int k_neighbors;

uniform vec2 inv_screen_size;
uniform mat4 inv_projection_matrix;

in vec3 uvd;
in vec2 screen_pos;

out vec2 radius_bounds;

void main() {
  float depth = texture(depth_sampler, uvd.xy).r * 2.0 - 1.0;
  float num_points = texture(num_points_grid_sampler, uvd.xy).r;
  num_points = max(1.0, num_points);

  float r_screen = sqrt((grid_area * k_neighbors) / (PI * num_points));

  vec4 unproj = inv_projection_matrix * vec4(r_screen * inv_screen_size, depth, 1.0);
  vec2 r_world = abs(unproj.xy / unproj.w);

  radius_bounds.x = 0.0;
  // radius_bounds.y = max(r_world.x, r_world.y) * 2.0;
  radius_bounds.y = clamp(max(r_world.x, r_world.y) * init_radius_scale, min_init_radius, max_init_radius);
  // radius_bounds.y = max(r_world.x, r_world.y) * 1.0;
}