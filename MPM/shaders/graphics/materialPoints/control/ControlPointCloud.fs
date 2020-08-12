#version 450 core

uniform vec4 pointCloudColor = vec4(1.0, 1.0, 0.0, 1.0);

out vec4 fragColor;

void main() {
    fragColor = pointCloudColor;
}