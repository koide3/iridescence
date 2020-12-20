#version 330
const float PI = 3.1415926535;

float schlick_fresnel(float f0, float f90, float cosine);

float normal_distribution_ggx(float roughness, vec3 N, vec3 H) {
  float roughness_sq = roughness * roughness;
  float dotNH = clamp(dot(N, H), 0.0, 1.0);
  float a = (1.0 - (1.0 - roughness_sq) *dotNH * dotNH);
  return roughness_sq / (2.0 * PI * a * a);
}

float masking_shadow_smith(float roughness, vec3 N, vec3 L, vec3 V) {
  float roughness_sq = roughness * roughness;
  float dotNV = clamp(dot(N, V), 0.0, 1.0);
  float dotNL = clamp(dot(N, L), 0.0, 1.0);
  float lv = 0.5 * (-1.0 + sqrt(1.0 + roughness_sq * (1.0 / (dotNV * dotNV) - 1.0)));
  float ll = 0.5 * (-1.0 + sqrt(1.0 + roughness_sq * (1.0 / (dotNL * dotNL) - 1.0)));
  return (1.0 / (1.0 + lv)) + (1.0 / (1.0 + ll));
}

float specular_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V) {
  vec3 H = normalize(V + L);
  float dotNV = clamp(dot(N, V), 0.0, 1.0);
  float dotNL = clamp(dot(N, L), 0.0, 1.0);
  float dotVH = clamp(dot(V, H), 0.0, 1.0);

  float d = normal_distribution_ggx(roughness, N, H);
  float g = masking_shadow_smith(roughness, N, L, V);
  float f = schlick_fresnel(albedo, 1.0, dotVH);

  return d * g * f / (4.0 * dotNV * dotNL);
}