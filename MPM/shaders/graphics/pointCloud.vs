#version 450 core

/*** HEADER ***/

void main() {

	// calculate radius
	gl_PointSize = 2.0;

	// only draw on the right half of the window screen
	dvec2 norm_pos = (2.0 * points[gl_VertexID].x - vec2(GRID_SIZE_X, GRID_SIZE_Y))/vec2(GRID_SIZE_X, GRID_SIZE_Y);
	norm_pos.x = norm_pos.x / 2.0;
	norm_pos.x = norm_pos.x + 0.5;

	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
}