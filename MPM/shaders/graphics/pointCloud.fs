#version 450 core

uniform vec3 pointCloudColor = vec3(1.0, 0.0, 0.0);

void main() {
	//float r = (dot(gl_PointCoord, gl_PointCoord) < 1.0) ? 1.0 : 0.0;
    gl_FragColor = vec4(pointCloudColor, 1.0);
}