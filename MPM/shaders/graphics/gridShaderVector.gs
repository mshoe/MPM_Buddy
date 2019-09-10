#version 450 core

in vec4 vs_nodeColor[];
in int gridNodeI[];
in int gridNodeJ[];

out vec4 gs_nodeColor;

layout (points) in;
layout (line_strip, max_vertices = 2) out;

uniform int selectedVector = 1;
uniform double maxGridVectorLength;
uniform double maxGridVectorVisualLength;

uniform double zoomFactor;

/*** HEADER ***/

void main() {

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 vector = dvec2(0.0, 0.0);

    if (selectedVector == 0) { // velocity
        vector = nodes[gridNodeI[0]][gridNodeJ[0]].v / grid_vec; // normalizing velocity?
    } else if (selectedVector == 1) { // acceleration
        double m = (nodes[gridNodeI[0]][gridNodeJ[0]].m == 0) ? 1.0 : nodes[gridNodeI[0]][gridNodeJ[0]].m;
        vector = nodes[gridNodeI[0]][gridNodeJ[0]].force / grid_vec / m;
    } else if (selectedVector == 2) { // force
        vector = nodes[gridNodeI[0]][gridNodeJ[0]].force / grid_vec;
    } else if (selectedVector == 3) { // residual velocity (rk)
        vector = nodes[gridNodeI[0]][gridNodeJ[0]].rk / grid_vec;
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