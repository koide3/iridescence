#version 330
uniform sampler2D color_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D occlusion_sampler;

const int max_num_lights = 10;

uniform vec3 view_point;
uniform float material_shininess;

uniform int num_lights;
uniform vec3 light_pos[max_num_lights];
uniform vec4 light_color[max_num_lights];

uniform vec4 ambient_light_color;

in vec2 texcoord;

layout(location = 0) out vec4 final_color;

vec4 lighting(int i, vec3 view_vec, vec3 frag_position, vec3 frag_normal) {
  vec3 light_vec = normalize(light_pos[i] - frag_position);
  float diffuse = max(dot(frag_normal, light_vec), 0.0);

  vec3 reflect_vec = reflect(-light_vec, frag_normal);
  float specular = pow(max(dot(reflect_vec, view_vec), 0.0), material_shininess);

  return (diffuse + specular) * light_color[i];
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

  vec3 view_vec = normalize(view_point);

  vec4 color = vec4(0.0);
  for(int i = 0; i < num_lights; i++) {
    color += lighting(i, view_vec, frag_position.xyz, frag_normal);
  }

  float occlusion = (1 - frag_occlusion * 2.0);

  final_color = color * frag_color * occlusion + ambient_light_color;
}