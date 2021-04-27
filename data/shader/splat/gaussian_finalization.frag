#version 330

uniform sampler2D num_points_sampler;
uniform sampler2D sum_points_sampler;
uniform sampler2D sum_cross1_sampler;
uniform sampler2D sum_cross2_sampler;

uniform int k_neighbors;
uniform int k_tolerance;

in vec2 texcoord;

out float num_points;
out vec3 mean;
out vec3 cov1;
out vec3 cov2;

void main() {
  float N = texture(num_points_sampler, texcoord).x;

  if(abs(N - k_neighbors) > k_tolerance + 0.5) {
    discard;
  }

  vec3 sum_points = texture(sum_points_sampler, texcoord).xyz;
  vec3 sum_cross1 = texture(sum_cross1_sampler, texcoord).xyz;  // c11, c12, c13
  vec3 sum_cross2 = texture(sum_cross2_sampler, texcoord).xyz;  // c22, c23, c33

  mat3 sum_cross;
  sum_cross[0] = vec3(sum_cross1[0], sum_cross1[1], sum_cross1[2]);
  sum_cross[1] = vec3(sum_cross1[1], sum_cross2[0], sum_cross2[1]);
  sum_cross[2] = vec3(sum_cross1[2], sum_cross2[1], sum_cross2[2]);

  num_points = N;
  mean = sum_points / N;

  mat3 cov = (sum_cross - outerProduct(mean, sum_points)) / N;
  cov1 = cov[0];
  cov2 = vec3(cov[1].y, cov[1].z, cov[2].z);
}