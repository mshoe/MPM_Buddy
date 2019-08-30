#version 450 core

uniform vec4 pointCloudColor = vec4(1.0, 0.0, 0.0, 1.0);

void main() {
	//float r = (dot(gl_PointCoord, gl_PointCoord) < 1.0) ? 1.0 : 0.0;
    gl_FragColor = pointCloudColor;
}