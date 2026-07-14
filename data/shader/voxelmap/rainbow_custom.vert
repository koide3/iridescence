uniform float voxel_resolution;   // Voxel resolution
in ivec3 voxel_coord;             // Voxel coordinates for instance rendering

#define OVERRIDE_VERTEX_POSITION
vec3 get_vertex_position(vec3 v) {
    return v + vec3(voxel_coord) * voxel_resolution;
}