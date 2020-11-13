#version 330
uniform sampler2D color_sampler;
uniform sampler2D depth_sampler;

const int num_samples = 32;

uniform float ao_radius;
uniform vec3 random_vectors[num_samples];
uniform mat4 projection_view_matrix;
uniform mat4 inv_projection_view_matrix;

in vec2 texcoord;

out vec4 color;

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
  color = texture(color_sampler, texcoord);

  float depth = texture(depth_sampler, texcoord).x * 2.0 - 1.0;
  if(depth > 1.0 - 1e-5) {
    return;
  }

  float occluded = 0.0;

  vec3 xyz = unproject(tex2uv(texcoord), depth);
  for(int i=0; i<num_samples; i++) {
    vec3 p = xyz + ao_radius * random_vectors[i];
    vec3 p_uvd = project(p);
    float p_depth = texture(depth_sampler, uv2tex(p_uvd.xy)).x * 2.0 - 1.0;

    if(p_depth < p_uvd.z) {
      occluded += clamp(1 - (p_uvd.z - p_depth) / ao_radius, 0.0, 1.0);
    }
  }

  float occlusion = clamp(2.0 * (occluded / num_samples - 0.5), 0.0, 1.0);
  color = color * (1 - occlusion);
}