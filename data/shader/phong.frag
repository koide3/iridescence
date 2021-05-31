#version 330
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 inv_view_matrix;
uniform mat4 projection_matrix;

uniform vec3 light_pos;
uniform vec4 light_color;
uniform vec4 ambient_color;

uniform float material_shininess;
uniform vec4 material_color;
uniform vec4 material_emission;

in vec3 frag_position;
in vec3 frag_normal;

out vec4 color;

vec4 phong() {
    vec3 light_vec = normalize(light_pos - frag_position);
    float diffuse = max(dot(normalize(frag_normal), light_vec), 0.0);

    vec3 view_vec = normalize(inv_view_matrix[3].xyz);
    vec3 reflect_vec = reflect(-light_vec, normalize(frag_normal));
    // vec3 ref = 2.0 * frag_normal * dot(light_vec, frag_normal) - light_vec;
    float specular = pow(max(dot(reflect_vec, view_vec), 0.0), material_shininess);

    return (diffuse + specular) * light_color + ambient_color;
}

void main() {
    gl_FragData[0] = phong() * material_color + material_emission;
    gl_FragData[1] = material_color;
    gl_FragData[2] = vec4(normalize(frag_normal), 1.0);
}