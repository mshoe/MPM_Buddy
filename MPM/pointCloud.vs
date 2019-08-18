#version 450 core

/* begin MPM shaders header */
const int GRID_SIZE_X = 32*4;
const int GRID_SIZE_Y = 32*4;

struct materialPoint {
	vec2 x;
	vec2 v;
	float m;
	float vol;
	mat2 B;
	mat2 F;
	mat2 P;
};

layout (std430, binding = 1) buffer pointCloud {
	readonly materialPoint points[];
};

uniform float dt = 1/60.0;
/* end MPM shaders header */

void main() {

	// calculate radius
	gl_PointSize = 2.0;

	vec2 norm_pos = (2.0 * points[gl_VertexID].x - vec2(GRID_SIZE_X, GRID_SIZE_Y))/vec2(GRID_SIZE_X, GRID_SIZE_Y);

    gl_Position = vec4(norm_pos, 0.0, 1.0);
}