#version 430

in vec3 vert_position;
in vec3 vert_face_layers;  // (top_layer, side_layer, bottom_layer)

out vec3 frag_vert_position;
out vec3 frag_face_layers;

void main() {
    frag_vert_position = vert_position;
    frag_face_layers = vert_face_layers;
}
