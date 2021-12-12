#version 330
uniform bool info_enabled;
uniform ivec4 info_values;
uniform int dynamic_object;

uniform bool normal_enabled;
uniform bool partial_rendering_enabled;

// colormode = 0 : rainbow (height encoding)
// colormode = 1 : material_color
// colormode = 2 : vert_color
// colormode = 3 : texture_color
uniform int color_mode;
uniform sampler2D texture_sampler;

in vec4 frag_color;
in vec2 frag_texcoord;
in vec3 frag_normal;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;
layout (location=2) out vec3 normal;
layout (location=3) out int dynamic_flag;

void main() {
    if(color_mode == 3) {
        color = texture(texture_sampler, frag_texcoord);
    } else {
        color = frag_color;
    }

    if(info_enabled) {
        info = info_values;
    }

    if(normal_enabled) {
        normal = normalize(frag_normal);
    }

    if(partial_rendering_enabled) {
        dynamic_flag = dynamic_object;
    }
}