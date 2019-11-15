#version 450 core

in flat vec4 vs_tripletColor;
out vec4 fragColor;

void main() {
    fragColor = vs_tripletColor;
}