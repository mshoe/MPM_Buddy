#version 450 core

uniform double zoomFactor;
uniform dvec2 zoomPoint;

/*** HEADER ***/

void main() {

	// calculate radius
	gl_PointSize = 2.0;
	uint pointID = gl_VertexID;

	// map position to (-1.0, 1.0)
	dvec2 pos = points[pointID].x - zoomPoint;
	pos *= zoomFactor;
	pos += zoomPoint;
	dvec2 norm_pos = (2.0*pos - dvec2(GRID_SIZE_X, GRID_SIZE_Y))/dvec2(GRID_SIZE_X, GRID_SIZE_Y);
	
	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
}