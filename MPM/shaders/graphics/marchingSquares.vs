#version 450 core

out vec4 vs_nodeColor;
out int gridNodeI;
out int gridNodeJ;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)
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