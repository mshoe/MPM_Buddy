#version 450 core

uniform vec2 iResolution;
uniform mat4 iCamera;
uniform vec4 iMouse;
uniform float iTime;

const vec3 camera_from = iCamera[3].xyz;
const vec3 camera_dir = -iCamera[2].xyz;
const vec3 camera_right = iCamera[0].xyz;
const vec3 camera_up = iCamera[1].xyz;


float circleSdf(vec3 p) {
	
}

void main() {
	
	vec3 color = vec3(1.0);
	
	gl_FragColor = vec4(color, 1.0);
}