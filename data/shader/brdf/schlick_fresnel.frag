#version 330

float schlick_fresnel(float f0, float f90, float cosine) {
  return f0 + (f90 - f0) * pow(1.0 - cosine, 5.0);
}