#version 450 core

in vec4 stressColor;

uniform bool visualizeEnergy = false;
uniform vec4 pointCloudColor = vec4(1.0, 0.0, 0.0, 1.0);

void main() {
	//float r = (dot(gl_PointCoord, gl_PointCoord) < 1.0) ? 1.0 : 0.0;
    if (visualizeEnergy) {
        gl_FragColor = stressColor;
    }
    else {
        gl_FragColor = pointCloudColor;
    }
}