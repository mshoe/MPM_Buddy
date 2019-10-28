#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 100) out;

layout (std430, binding = 4) buffer pwLineVertices {
	dvec2 vertices[];
};

uniform dvec4 iMouse;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)

uniform bool lastVertexMouse = false;

//uniform dvec2 zoomPoint;
//uniform double zoomFactor;

/*** HEADER ***/


void main() {
    int numVertices = vertices.length();
    // Only calling this shader when numVertices > 1, managed in c++ code

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

    for (int i = 0; i < numVertices; i++) {
        dvec2 pos = vertices[i] / grid_vec;

        dvec2 norm_pos;

        norm_pos.x = mix(left_border, right_border, pos.x);
	    norm_pos.y = mix(bottom_border, top_border, pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // one more for mouse


    if (lastVertexMouse) {
        dvec2 norm_mouse;
        norm_mouse.x = mix(left_border, right_border, iMouse.x);
        norm_mouse.y = mix(bottom_border, top_border, iMouse.y);
        gl_Position = vec4(float(norm_mouse.x), float(norm_mouse.y), 0.0, 1.0);
        EmitVertex();
    }

    // gl_Position = vec4(0.7, 0.7, 0.0, 1.0);
    // EmitVertex();

    // gl_Position = vec4(0.9, 0.9, 0.0, 1.0);
    // EmitVertex();

    EndPrimitive();
}