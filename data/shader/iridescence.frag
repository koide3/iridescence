#version 330
uniform sampler2D color_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D occlusion_sampler;

uniform vec3 view_point;
uniform vec3 light_pos;
uniform vec4 light_color;
uniform float material_shininess;

in vec2 texcoord;

layout (location=0) out vec4 final_color;

float fresnel(vec3 normal, vec3 pos) {
  float f0 = 0.05;
  vec3 v = normalize(pos - view_point);
  return clamp(f0 + f0 * pow(1.0 - dot(normal, v), 5.0), 0.0, 1.0);
}

void main() {
  vec4 frag_color = texture(color_sampler, texcoord);
  vec4 frag_position = texture(position_sampler, texcoord);
  vec3 frag_normal = normalize(texture(normal_sampler, texcoord).xyz);
  float frag_occlusion = texture(occlusion_sampler, texcoord).x;

  if(frag_position.w > 1.0 - 1e-5) {
    final_color = frag_color;
    return;
  }

  vec3 light_vec = normalize(light_pos - frag_position.xyz);
  float diffuse = max(dot(frag_normal, light_vec), 0.0) * 0.8 + 0.5;

  float f = 1.0 - fresnel(frag_normal, frag_position.xyz);

  vec3 view_vec = normalize(view_point);
  vec3 reflect_vec = reflect(-light_vec, frag_normal);
  float specular = pow(max(dot(reflect_vec, view_vec), 0.0), material_shininess);

  vec4 base_color = (diffuse) * frag_color * (1 - frag_occlusion);
  vec4 normal_color = vec4(abs(frag_normal) * 0.25, 1.0);

  final_color = base_color + normal_color * (f + 0.2);

  return;
}