
#version 330
const float PI = 3.1415926535;

float diffuse_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V) {
  return albedo / PI;
}