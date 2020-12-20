#version 330
const float PI = 3.1415926535;

float schlick_fresnel(float f0, float f90, float cosine);

float diffuse_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V) {
  float bias = mix(0.0, 0.5, roughness);
  float factor = mix(1.0, 1.0 / 1.51, roughness);

  vec3 H = normalize(L + V);
  float dotLH = clamp(dot(L, H), 0.0, 1.0);
  float dotNL = clamp(dot(N, L), 0.0, 1.0);
  float dotNV = clamp(dot(N, V), 0.0, 1.0);
  float Fd90 = bias + 2.0 * dotLH * dotLH * roughness;
  float FL = schlick_fresnel(1.0, Fd90, dotNL);
  float FV = schlick_fresnel(1.0, Fd90, dotNV);
  return (albedo * FL * FV * factor) / PI;
}