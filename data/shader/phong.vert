#version 330
uniform mat4 model_matrix;

uniform mat4 view_matrix;
uniform mat4 inv_view_matrix;
uniform mat4 projection_matrix;

in vec3 vert_position;
in vec3 vert_normal;

out vec3 frag_position;
out vec3 frag_normal;

void main() {
    vec4 world_position = model_matrix * vec4(vert_position, 1.0);
    gl_Position = projection_matrix * view_matrix * world_position;

    frag_position = world_position.xyz;
    frag_normal = mat3(model_matrix) * vert_normal;
}