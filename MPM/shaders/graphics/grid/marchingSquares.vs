#version 450 core

out vec4 vs_nodeColor;
out int gridNodeI;
out int gridNodeJ;

uniform double zoomFactor;
uniform dvec2 zoomPoint;

/*** HEADER ***/

// vertices will be the GRID_SIZE_X * GRID_SIZE_Y grid nodes.
// Will need to break them up into (i, j) for indexing particleGrid struct.

// grid spacing is 1


void main() {

	// calculate radius
    
//	gl_PointSize = 1.0;
    vs_nodeColor = vec4(1.0, 1.0, 1.0, 1.0);

    int nodei = int(floor(gl_VertexID / GRID_SIZE_Y));
    int nodej = int(mod(gl_VertexID, GRID_SIZE_Y));

    gridNodeI = nodei;
    gridNodeJ = nodej;
	

	// gl_Position needs to take float, not double
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
}