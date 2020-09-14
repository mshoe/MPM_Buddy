#version 450 core

in vec4 vs_nodeColor[];
in int gridNodeI[];
in int gridNodeJ[];

out vec4 gs_nodeColor;

layout (points) in;
layout (line_strip, max_vertices = 2) out;


uniform double zoomFactor;

uniform uint selectedVector = 3;
uniform double maxGridVectorLength;
uniform double maxGridVectorVisualLength;

const uint VIS_GRID_MOMENTUM = 0;
const uint VIS_GRID_VELOCITY = 1;
const uint VIS_GRID_ACCELERATION = 2;
const uint VIS_GRID_FORCE = 3;
const uint VIS_GRID_RESIDUAL_VELOCITY = 4;
const uint VIS_GRID_NODAL_ACCELERATION = 5;

/*** HEADER ***/



void main() {

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 vector = dvec2(0.0, 0.0);


    gridNode node;
    GetNode(gridNodeI[0], gridNodeJ[0], node);


    if (selectedVector == VIS_GRID_MOMENTUM) { // momentum
        vector = node.momentum / grid_vec; // normalizing momentum
    } else if (selectedVector == VIS_GRID_VELOCITY) { // velocity
        vector = node.v / grid_vec; // normalizing velocity?
    } else if (selectedVector == VIS_GRID_ACCELERATION) { // acceleration
        double m = (node.m == 0) ? 1.0 : node.m;
        vector = node.force / grid_vec / m;
    } else if (selectedVector == VIS_GRID_FORCE) { // force
        vector = node.force / grid_vec;
    } else if (selectedVector == VIS_GRID_RESIDUAL_VELOCITY) { // residual velocity (rk)
        vector = node.rk / grid_vec;
    } else if (selectedVector == VIS_GRID_NODAL_ACCELERATION) { // nodal accelerations
        vector = node.nodalAcceleration / grid_vec;
    }

    double vLength = clamp(length(vector), 0.0, maxGridVectorLength);
    double normalizedVLength = vLength / maxGridVectorLength;
    normalizedVLength = normalizedVLength * normalizedVLength; // square to get better distribution
    vector = normalize(vector) * normalizedVLength * maxGridVectorVisualLength;

    // coloring
	vec3 maxColor = vec3(1.0, 0.0, 0.0);
	vec3 midColor = vec3(0.0, 1.0, 0.0);
	vec3 minColor = vec3(0.0, 0.0, 1.0);

    vec4 nodeColor;
	if (normalizedVLength > 0.5) {
		normalizedVLength = 2.0 * vLength - 1.0;
		nodeColor = vec4(normalizedVLength * maxColor + (1.0 - normalizedVLength) * midColor, 1.0);
	} else {
		normalizedVLength = 2.0 * normalizedVLength;
		nodeColor = vec4(normalizedVLength * midColor + (1.0 - normalizedVLength) * minColor, 1.0);
	}




    gl_Position = gl_in[0].gl_Position; // + vec4(-0.1, 0.0, 0.0, 0.0);
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = nodeColor;
    EmitVertex();

    float v_scaling = 1.0 / float(GRID_SIZE_X);
    gl_Position = gl_in[0].gl_Position + v_scaling * vec4( float(vector.x), float(vector.y), 0.0, 0.0) * float(zoomFactor);
    gs_nodeColor = nodeColor;
    EmitVertex();
    
    EndPrimitive();
}