#version 330
uniform bool info_enabled;
uniform ivec4 info_values;

in vec4 frag_color;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;

void main() {
    color = frag_color;

    if(info_enabled) {
        info = info_values;
    }
}