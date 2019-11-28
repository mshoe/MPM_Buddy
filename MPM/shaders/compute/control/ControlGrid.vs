#version 450 core

out vec4 vs_nodeColor;
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

    double nodeMass = nodes[nodei][nodej].m;

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // norm_pos mapped to (0, 1)
    dvec2 norm_node_pos = dvec2(nodei, nodej) / grid_vec;

    // get normalized zoomPoint;
    dvec2 norm_zoomPoint = zoomPoint / grid_vec;
    norm_node_pos -= norm_zoomPoint;
    norm_node_pos *= zoomFactor;
    norm_node_pos += norm_zoomPoint;

	// // map norm_pos to the new borders;

    // map to opengl window space (-1.0, 1.0)
	norm_node_pos.x = mix(-1.0, 1.0, norm_node_pos.x);
	norm_node_pos.y = mix(-1.0, 1.0, norm_node_pos.y);
	

	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_node_pos.x), float(norm_node_pos.y), 0.0, 1.0);
}