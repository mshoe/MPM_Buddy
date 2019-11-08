#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 256) out;
// 36320 is max

layout (std430, binding = 3) buffer polygonVertices {
	dvec2 vertices[];
};

uniform dvec2 polygonCenter;

uniform dvec4 mouse;
uniform bool lastVertexMouse = false;
uniform dvec2 zoomPoint;
uniform double zoomFactor;

/*** HEADER ***/


void main() {
    int numVertices = vertices.length();
    // Only calling this shader when numVertices > 1, managed in c++ code

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    for (int i = 0; i < numVertices; i++) {
        dvec2 pos = polygonCenter + vertices[i];
        pos -= zoomPoint;
        pos *= zoomFactor;
        pos += zoomPoint;

        dvec2 norm_pos = pos / grid_vec;

        norm_pos.x = mix(-1.0, 1.0, norm_pos.x);
	    norm_pos.y = mix(-1.0, 1.0, norm_pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // one more for mouse


    if (lastVertexMouse) {
        dvec2 norm_mouse = mouse.xy;
        norm_mouse -= zoomPoint;
        norm_mouse *= zoomFactor;
        norm_mouse += zoomPoint;
        norm_mouse /= grid_vec;
        norm_mouse.x = mix(-1.0, 1.0, norm_mouse.x);
        norm_mouse.y = mix(-1.0, 1.0, norm_mouse.y);
        gl_Position = vec4(float(norm_mouse.x), float(norm_mouse.y), 0.0, 1.0);
        EmitVertex();
    } else {
        dvec2 pos = polygonCenter + vertices[0];
        pos -= zoomPoint;
        pos *= zoomFactor;
        pos += zoomPoint;

        dvec2 norm_pos = pos / grid_vec;

        norm_pos.x = mix(-1.0, 1.0, norm_pos.x);
	    norm_pos.y = mix(-1.0, 1.0, norm_pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // gl_Position = vec4(0.7, 0.7, 0.0, 1.0);
    // EmitVertex();

    // gl_Position = vec4(0.9, 0.9, 0.0, 1.0);
    // EmitVertex();

    EndPrimitive();
}