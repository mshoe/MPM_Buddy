#version 450 core

in vec4 gs_nodeColor;

out vec4 fragColor;

void main() {
	fragColor = gs_nodeColor;
}