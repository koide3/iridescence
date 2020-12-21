#version 330

uniform sampler2D iridescence_sampler;

vec4 iridescence(vec3 N, vec3 L, vec3 V) {
  float theta_L = clamp(dot(L, N), 0.0, 1.0);
  float theta_V = clamp(dot(V, N), 0.0, 1.0);
  return texture(iridescence_sampler, vec2(theta_V, theta_L));

  /*
  vec3 tangent = normalize(cross(N, vec3(1.0, 0.0, 0.0)));

  vec3 L_ = normalize(L - dot(L, N) * N);
  vec3 V_ = normalize(V - dot(V, N) * N);

  float theta_L = clamp(dot(tangent, L_), -1.0, 1.0);
  float theta_V = clamp(dot(tangent, V_), -1.0, 1.0);

  return texture(iridescence_sampler, vec2(theta_V * 0.5 + 0.5, theta_L * 0.5 + 0.5));
  */
}