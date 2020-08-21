#version 330
uniform sampler2D color_sampler;
uniform sampler2D depth_sampler;

uniform vec3 random_vectors[512];

in vec2 texcoord;

out vec4 color;

void main() {
  int occluded = 0;
  int num_samples = 512;

  // lazy SSAO implementation
  float depth = texture(depth_sampler, texcoord).x;
  for(int i = 0; i < num_samples; i++) {
    vec3 v = random_vectors[i] * (1 - depth);
    float d = texture(depth_sampler, texcoord + v.xy).x;
    occluded += d < depth - v.z ? 1 : 0;
  }

  float w = 1.0 - occluded / float(num_samples);
  color = w * texture(color_sampler, texcoord);
}