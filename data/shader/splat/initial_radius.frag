#version 330

const float PI = 3.1415926535;

uniform sampler2D num_points_grid_sampler;

uniform int grid_area;
uniform int k_neighbors;

uniform vec2 inv_screen_size;
uniform mat4 inv_projection_matrix;

in vec3 uvd;
in vec2 screen_pos;

out vec2 radius_bounds;

void main() {
  float num_points = texture(num_points_grid_sampler, uvd.xy).r;
  num_points = max(1.0, num_points);

  float r_screen = sqrt((grid_area * k_neighbors) / (PI * num_points));

  float depth = uvd.z * 2.0 - 1;
  vec4 unproj = inv_projection_matrix * vec4(r_screen * inv_screen_size, depth, 1.0);
  vec2 r_world = abs(unproj.xy / unproj.w);

  radius_bounds.x = 0.0;
  // radius_bounds.y = max(r_world.x, r_world.y) * 2.0;
  radius_bounds.y = max(max(r_world.x, r_world.y) * 2.0, 0.1);
}