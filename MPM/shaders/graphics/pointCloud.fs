#version 450 core

in vec4 gs_speedColor;
in vec4 gs_stressColor;

uniform bool visualizeSpeed = false;
uniform bool visualizeEnergy = false;
uniform vec4 pointCloudColor = vec4(1.0, 0.0, 0.0, 1.0);

out vec4 fragColor;

void main() {
	//float r = (dot(gl_PointCoord, gl_PointCoord) < 1.0) ? 1.0 : 0.0;
    if (visualizeSpeed) {
        fragColor = gs_speedColor;
    }
    else if (visualizeEnergy) {
        fragColor = gs_stressColor;
    }
    else {
        fragColor = pointCloudColor;
    }
}