#version 130
uniform sampler2D texture_sampler;

in vec2 texcoord;
out vec4 color;

void main() {
    color = texture(texture_sampler, texcoord);
}