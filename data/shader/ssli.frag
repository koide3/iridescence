#version 330
const float PI = 3.1415926535;
const int max_num_lights = 128;

uniform sampler2D color_sampler;
uniform sampler2D position_sampler;
uniform sampler2D normal_sampler;
uniform sampler2D occlusion_sampler;
uniform sampler2D iridescence_sampler;

uniform vec3 view_point;

uniform float albedo;
uniform float roughness;

uniform int num_lights;
uniform int light_directional[max_num_lights];
uniform float light_range[max_num_lights];
uniform vec2 light_attenuation[max_num_lights];
uniform vec3 light_pos[max_num_lights];
uniform vec4 light_color[max_num_lights];

uniform vec4 ambient_light_color;

in vec2 texcoord;

layout(location = 0) out vec4 final_color;

float diffuse_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V);
float specular_brdf(float albedo, float roughness, vec3 N, vec3 L, vec3 V);
float occlusion(float frag_occlusion);
vec4 iridescence(vec3 N, vec3 L, vec3 V);

vec4 lighting(int i, vec3 frag_position, vec3 frag_normal, vec3 view_point) {
  vec3 light_vec = -light_pos[i];
  float distance = 0.0;

  if(light_directional[i] == 0) {
    light_vec = light_pos[i] - frag_position;
    distance = length(light_vec);

    if(distance > light_range[i]) {
      return vec4(0.0);
    }
  }

  vec3 N = frag_normal;
  vec3 L = normalize(light_vec);
  vec3 V = normalize(view_point - frag_position);

  float diffuse = clamp(diffuse_brdf(albedo, roughness, N, L, V), 0.0, 1.0);
  float specular = clamp(specular_brdf(albedo, roughness, N, L, V), 0.0, 1.0);

  float cosine = clamp(dot(N, L), 0.0, 1.0);
  float attenuation = 1.0 / (1.0 + light_attenuation[i][0] * distance + light_attenuation[i][1] * distance * distance);
  return cosine * attenuation * (diffuse + specular) * light_color[i] * iridescence(N, L, V);
}

void main() {
  vec4 frag_color = texture(color_sampler, texcoord);
  vec4 frag_position = texture(position_sampler, texcoord);
  vec3 frag_normal = normalize(texture(normal_sampler, texcoord).xyz);
  float frag_occlusion = texture(occlusion_sampler, texcoord).x;

  /*
  if(frag_position.w > 1.0 - 1e-5) {
    final_color = frag_color;
    return;
  }
  */

  vec4 color = vec4(0.0);
  for(int i = 0; i < num_lights; i++) {
    color += lighting(i, frag_position.xyz, frag_normal, view_point);
  }

  float openness = 1.0 - occlusion(frag_occlusion);
  final_color = color * frag_color * openness + ambient_light_color;
}