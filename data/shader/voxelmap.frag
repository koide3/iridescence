#version 430

uniform bool info_enabled;
uniform ivec4 info_values;
uniform bool normal_enabled;
uniform bool partial_rendering_enabled;
uniform int dynamic_object;

uniform sampler2DArray texture_array;

in vec3 frag_normal;
in vec3 frag_texcoord3d;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;
layout (location=2) out vec3 normal;
layout (location=3) out int dynamic_flag;

void main() {
    color = texture(texture_array, frag_texcoord3d);

    if (color.a < 0.1) {
        discard;
    }

    if (info_enabled) {
        info = info_values;
    }

    if (normal_enabled) {
        normal = normalize(frag_normal);
    }

    if (partial_rendering_enabled) {
        dynamic_flag = dynamic_object;
    }
}
