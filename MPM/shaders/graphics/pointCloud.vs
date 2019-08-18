#version 450 core

/*** HEADER ***/

void main() {

	// calculate radius
	gl_PointSize = 2.0;

	vec2 norm_pos = (2.0 * points[gl_VertexID].x - vec2(GRID_SIZE_X, GRID_SIZE_Y))/vec2(GRID_SIZE_X, GRID_SIZE_Y);

    gl_Position = vec4(norm_pos, 0.0, 1.0);
}