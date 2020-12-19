#version 330
const float PI = 3.1415926535;

float specular_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V) {
  float power = pow(1.0 - roughness, 2.0) * 10.0;
  vec3 H = normalize(L + V);
  float dotNH = clamp(dot(N, H), 0.0, 1.0);
  float norm = (power + 2.0) / (2.0 * PI);
  return albedo * pow(dotNH, power) * norm;
}