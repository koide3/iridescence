#version 330
uniform sampler2D color_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D occlusion_sampler;

in vec2 texcoord;

layout(location = 0) out vec4 final_color;

void main() {
  vec4 frag_color = texture(color_sampler, texcoord);
  vec4 frag_position = texture(position_sampler, texcoord);
  vec3 frag_normal = normalize(texture(normal_sampler, texcoord).xyz);
  float frag_occlusion = texture(occlusion_sampler, texcoord).x;

  if(frag_position.w > 1.0 - 1e-5) {
    final_color = frag_color;
    return;
  }

  final_color = frag_color * (1 - clamp(frag_occlusion * 2.0 - 0.4, 0.0, 1.0));
}