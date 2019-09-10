#version 450 core

out vec4 vs_nodeColor;
out int gridNodeI;
out int gridNodeJ;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)
uniform double zoomFactor;
uniform dvec2 zoomPoint;

uniform bool viewGridMass;
uniform double maxNodeMassClamp;
uniform double minNodeMassClamp;
uniform double maxNodeMassPointSize;
uniform double minNodeMassPointSize;

uniform int gridPointSizeScalingOption;

uniform bool nodeGraphicsActive;
uniform int selectedNodeI;
uniform int selectedNodeJ;

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



    if (viewGridMass) {
        double normNodeMass = (nodeMass - minNodeMassClamp)/ (maxNodeMassClamp - minNodeMassClamp);
        normNodeMass = clamp(normNodeMass, 0.0, 1.0);
        double scaledNormNodeMass;
        if (gridPointSizeScalingOption == -1) {
            scaledNormNodeMass = sqrt(sqrt(normNodeMass));
        } else if (gridPointSizeScalingOption == 0) {
            scaledNormNodeMass = sqrt(normNodeMass);
        } else if (gridPointSizeScalingOption == 1) {
            scaledNormNodeMass = normNodeMass;
        } else if (gridPointSizeScalingOption == 2) {
            scaledNormNodeMass = normNodeMass * normNodeMass;
        } else if (gridPointSizeScalingOption == 3) {
            scaledNormNodeMass = normNodeMass * normNodeMass * normNodeMass;
        }
        gl_PointSize = float(mix(minNodeMassPointSize, maxNodeMassPointSize, sqrt(scaledNormNodeMass)));
    } else {
        gl_PointSize = 1.0;
    }

    if (nodeGraphicsActive) {
        if (nodei == selectedNodeI && nodej == selectedNodeJ) {
            vs_nodeColor = vec4(0.0, 1.0, 0.0, 1.0);
            gl_PointSize = 10.0;
        }
    }

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // norm_pos mapped to (0, 1)
    dvec2 norm_node_pos = dvec2(nodei, nodej) / grid_vec;

    // get normalized zoomPoint;
    dvec2 norm_zoomPoint = zoomPoint / grid_vec;
    norm_node_pos -= norm_zoomPoint;
    norm_node_pos *= zoomFactor;
    norm_node_pos += norm_zoomPoint;


	// original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

	// map norm_pos to the new borders;

	norm_node_pos.x = mix(left_border, right_border, norm_node_pos.x);
	norm_node_pos.y = mix(bottom_border, top_border, norm_node_pos.y);
	

	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_node_pos.x), float(norm_node_pos.y), 0.0, 1.0);
}