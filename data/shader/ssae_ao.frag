#version 330
uniform sampler2D depth_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D randomization_sampler;

const int num_samples = 16;

uniform float ao_radius;
uniform vec3 random_vectors[num_samples];
uniform mat4 projection_view_matrix;
uniform vec2 randomization_coord_scale;

in vec2 texcoord;

layout(location = 0) out float frag_occlusion;
layout(location = 1) out vec4 frag_color;

vec2 uv2tex(vec2 uv) {
  return 0.5 * uv + 0.5;
}

vec3 project(vec3 xyz) {
  vec4 uvd = projection_view_matrix * vec4(xyz, 1.0);
  return uvd.xyz / uvd.w;
}

void main() {
  float depth = texture(depth_sampler, texcoord).x * 2.0 - 1.0;
  if(depth > 1.0 - 1e-5) {
    frag_occlusion = 0.0;
    frag_color = vec4(1.0);
    return;
  }

  vec3 xyz = texture(position_sampler, texcoord).xyz;
  vec3 normal = texture(normal_sampler, texcoord).xyz;
  vec3 noise = normalize(texture(randomization_sampler, texcoord * randomization_coord_scale).xyz);

  float occluded = 0.0;
  for(int i = 0; i < num_samples; i++) {
    vec3 v = reflect(random_vectors[i], noise);
    if(dot(v, normal) < 0.0) {
      v = -v;
    }

    vec3 p = xyz + ao_radius * v;
    vec3 p_uvd = project(p);
    vec2 p_coord = uv2tex(p_uvd.xy);

    float p_depth = texture(depth_sampler, p_coord).x * 2.0 - 1.0;

    if(p_depth > p_uvd.z) {
      continue;
    }

    float w = clamp(1 - (p_uvd.z - p_depth) / ao_radius, 0.0, 1.0);
    occluded += w;
  }

  frag_occlusion = occluded / num_samples;
  frag_color = vec4(1 - frag_occlusion);
  return;
}