#version 450 core

in vec4 vs_stressColor[];
in uint pointID[];
out vec4 gs_stressColor;

layout (points) in;
layout (line_strip, max_vertices = 2) out;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)
uniform double zoomFactor;
uniform dvec2 zoomPoint;

uniform int selectedVector = 0;
/*** HEADER ***/

void main() {

	// original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	// double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	// double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	// double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	// double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

    // First emit vertex at the original position
    gl_Position = gl_in[0].gl_Position;
    gs_stressColor = vs_stressColor[0];
    EmitVertex();

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 vector;

    if (selectedVector == 0) {
        vector = points[pointID[0]].v / grid_vec; // normalizing velocity?
    }

    float v_scaling = 0.1;

    gl_Position = gl_in[0].gl_Position + v_scaling * vec4( float(vector.x), float(vector.y), 0.0, 0.0);
    gs_stressColor = vs_stressColor[0];
    EmitVertex();
    
    EndPrimitive();
}