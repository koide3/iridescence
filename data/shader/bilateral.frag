#version 330
uniform sampler2D color_sampler;
uniform sampler2D position_sampler;

uniform vec2 inv_frame_size;
uniform vec2 filter_direction;

uniform bool second_pass;
uniform int half_kernel_size;
uniform float sigma_x;
uniform float sigma_d;

in vec2 texcoord;

layout (location=0) out vec4 frag_color;

void main() {
  vec4 center = texture(position_sampler, texcoord);
  if(center.w > 1.0 - 1e-5) {
    frag_color = vec4(vec3(0.0), 1.0);
    return;
  }

  vec4 sum = vec4(0.0);
  for(int i=-half_kernel_size; i<=half_kernel_size; i++) {
    vec2 coord = texcoord + inv_frame_size * filter_direction * i;
    vec4 color = texture(color_sampler, coord);
    vec4 pos = texture(position_sampler, coord);

    float d = length(pos.xyz - center.xyz);

    float wx = exp(-(i * i) / (2.0 * sigma_x));
    float wd = exp(-(d * d) / (2.0 * sigma_d));
    float w = wx * wd / (2 * half_kernel_size + 1);

    sum.w += w * color.w;
    sum.xyz += w * color.xyz;
  }

  frag_color = second_pass ? sum / sum.w : sum;
}