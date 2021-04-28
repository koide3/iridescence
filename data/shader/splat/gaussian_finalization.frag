#version 330

uniform sampler2D radius_bounds_sampler;
uniform sampler2D num_points_sampler;
uniform sampler2D sum_points_sampler;
uniform sampler2D sum_cross1_sampler;
uniform sampler2D sum_cross2_sampler;

uniform int k_neighbors;
uniform int k_tolerance;
uniform vec3 view_point;

in vec3 uvd;
in vec2 screen_pos;
// in vec2 texcoord;

out vec3 lengths;
out vec3 normal;
out vec3 minor_tangent;
out vec3 major_tangent;

void compute_eigen(vec3 c1, vec3 c2, out vec3 eval, out mat3 evec);
void sort_eigen(inout vec3 eval, inout mat3 evec);

void main() {
  vec2 texcoord = uvd.xy;
  float k = texture(num_points_sampler, texcoord).x;

  if(abs(k - k_neighbors) > k_tolerance + 0.5) {
    discard;
  }

  vec3 sum_points = texture(sum_points_sampler, texcoord).xyz;
  vec3 sum_cross1 = texture(sum_cross1_sampler, texcoord).xyz;  // c11, c12, c13
  vec3 sum_cross2 = texture(sum_cross2_sampler, texcoord).xyz;  // c22, c23, c33

  mat3 sum_cross;
  sum_cross[0] = vec3(sum_cross1[0], sum_cross1[1], sum_cross1[2]);
  sum_cross[1] = vec3(sum_cross1[1], sum_cross2[0], sum_cross2[1]);
  sum_cross[2] = vec3(sum_cross1[2], sum_cross2[1], sum_cross2[2]);

  vec3 mean = sum_points / k;

  mat3 cov = (sum_cross - outerProduct(mean, sum_points)) / k;
  vec3 c1 = cov[0];
  vec3 c2 = vec3(cov[1].y, cov[1].z, cov[2].z);

  vec3 eval;
  mat3 evec;
  compute_eigen(c1, c2, eval, evec);
  sort_eigen(eval, evec);

  float r_k = texture(radius_bounds_sampler, uvd.xy).x;
  float r_splat = 2.0 * sqrt(r_k * r_k / k);

  lengths = r_splat * eval / eval.z;
  normal = evec[0];

  if(dot(view_point - mean, normal) < 0.0) {
    normal = -normal;
  }

  minor_tangent = evec[1];
  major_tangent = evec[2];
}