#version 450 core

/* begin MPM shaders header */
const int GRID_SIZE_X = 32*4;
const int GRID_SIZE_Y = 32*4;

uniform vec2 iResolution;
uniform mat4 iCamera;
uniform vec4 iMouse;
uniform float iTime;

const vec3 camera_from = iCamera[3].xyz;
const vec3 camera_dir = -iCamera[2].xyz;
const vec3 camera_right = iCamera[0].xyz;
const vec3 camera_up = iCamera[1].xyz;


	
void main() {
	gl_FragColor = vec4(vec3(0.0), 1.0);
}