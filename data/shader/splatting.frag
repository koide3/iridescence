#version 330

uniform bool info_enabled;
uniform bool normal_enabled;
uniform bool partial_rendering_enabled;
uniform bool texture_enabled;

uniform ivec4 info_values;
uniform int dynamic_object;

// colormode = 0 : rainbow (height encoding)
// colormode = 1 : material_color
// colormode = 2 : vert_color
// colormode = 3 : texture_color
uniform int color_mode;
uniform sampler2D texture_sampler;

in FragmentData {
  vec2 texcoord;
  vec3 position;
  vec3 normal;
  vec4 color;
} fragment_in;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;
layout (location=2) out vec3 normal;
layout (location=3) out int dynamic_flag;

void main() {
  if (texture_enabled) {
    color = texture(texture_sampler, fragment_in.texcoord.xy);
    if (color.a < 0.01) {
      discard;
    }
  } else {
    float r = length(2.0 * fragment_in.texcoord.xy - vec2(1.0));
    if (r > 1.0) {
      discard;
    }
    color = vec4(1.0);
  }

  if (color_mode != 3) {
    color = color * fragment_in.color;
  }

  if (color.a < 0.01) {
    discard;
  }

  if(info_enabled) {
    info = info_values;
  }

  if(normal_enabled) {
    normal = normalize(fragment_in.normal);
  }

  if(partial_rendering_enabled) {
    dynamic_flag = dynamic_object;
  }
}