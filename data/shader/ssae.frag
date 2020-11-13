#version 330
uniform sampler2D color_sampler;
uniform sampler2D depth_sampler;

const int num_samples = 32;

uniform float ao_radius;
uniform float ne_radius;
uniform vec3 random_vectors[num_samples];

uniform vec3 view_point;
uniform mat4 projection_view_matrix;
uniform mat4 inv_projection_view_matrix;

in vec2 texcoord;

layout (location=0) out vec3 frag_position;
layout (location=1) out vec3 frag_normal;
layout (location=2) out float frag_occlusion;

vec2 tex2uv(vec2 texcoord) {
  return 2.0 * (texcoord - vec2(0.5, 0.5));
}

vec2 uv2tex(vec2 uv) {
  return 0.5 * uv + 0.5;
}

vec3 project(vec3 xyz) {
  vec4 uvd = projection_view_matrix * vec4(xyz, 1.0);
  return uvd.xyz / uvd.w;
}

vec3 unproject(vec2 uv, float depth) {
  vec4 xyzw = inv_projection_view_matrix * vec4(uv, depth, 1.0);
  return xyzw.xyz / xyzw.w;
}


void main() {
  float depth = texture(depth_sampler, texcoord).x * 2.0 - 1.0;
  if(depth >= 1.0 - 1e-5) {
    return;
  }

  float occluded = 0;
  float sum_w = 0;
  vec3 sum_pt = vec3(0);
  mat3 sum_cov = mat3(0);

  vec3 xyz = unproject(tex2uv(texcoord), depth);
  frag_position = xyz;

  for(int i=0; i<num_samples; i++) {
    vec3 p = xyz + ao_radius * random_vectors[i];
    vec3 p_uvd = project(p);
    float p_depth = texture(depth_sampler, uv2tex(p_uvd.xy)).x * 2.0 - 1.0;
    if(p_depth >= 1.0 - 1e-6) {
      continue;
    }

    if(p_depth < p_uvd.z) {
      occluded += clamp(1 - (p_uvd.z - p_depth) / ao_radius, 0.0, 1.0);
    }

    vec3 pt = unproject(p_uvd.xy, p_depth);
    float w = clamp(1 - length(pt - xyz) / ne_radius, 0.0, 1.0);
    // float w = clamp(1 - exp(-length(pt - xyz) * 1e2), 0.0, 1.0);
    sum_w += w;
    sum_pt += w * pt;
    sum_cov += w * outerProduct(pt, pt);
  }

  frag_occlusion = occluded / float(num_samples);
  // float occlusion = clamp(2.0 * (occluded / float(num_samples) - 0.54), 0.0, 1.0);

  vec3 mean = sum_pt / sum_w;
  mat3 cov = (sum_cov - outerProduct(sum_pt, mean));
  mat3 cov_inv = inverse(cov + 1e-3 * mat3(1.0));

  vec3 normal = normalize(view_point - mean);
  for(int i=0; i<5; i++) {
    normal = normalize(cov_inv * normal);
  }

  frag_normal = normal;
}