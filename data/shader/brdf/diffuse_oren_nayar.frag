#version 330
const float PI = 3.1415926535;

float diffuse_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V) {
  float dotNL = clamp(dot(N, L), 0.0, 1.0);
  float dotNV = clamp(dot(N, V), 0.0, 1.0);

  float roughness_sq = roughness * roughness;
  float a = 1.0 - 0.5 * roughness_sq / (roughness_sq + 0.33);
  float b = 0.45 * roughness_sq / (roughness_sq + 0.09);
  float cos_phi = dot(normalize(V - dotNV * N), normalize(L - dotNL * N));
  float sin_NV = sqrt(1.0 - dotNV * dotNV);
  float sin_NL = sqrt(1.0 - dotNL * dotNL);
  float s = dotNV < dotNL ? sin_NV : sin_NL;
  float t = dotNV > dotNL ? sin_NV / dotNV : sin_NL / dotNL;
  return albedo * (a + b * cos_phi * s * t) / PI;
}