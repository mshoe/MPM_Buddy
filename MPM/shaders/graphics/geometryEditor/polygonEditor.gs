#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 100) out;

uniform dvec2 polygonCenter;
uniform dvec2 windowSize; // w.r.t. to grid_vec = (128, 128)
layout (std430, binding = 3) buffer polygonVertices {
	dvec2 vertices[];
};

uniform dvec2 norm_mouse; // this should be the mouse coords w.r.t. this window (managed in C++ code)

uniform bool lastVertexMouse = false;

/*** HEADER ***/

void main() {
    int numVertices = vertices.length();
    // Only calling this shader when numVertices > 1, managed in c++ code

    for (int i = 0; i < numVertices; i++) {
        dvec2 pos = (polygonCenter + vertices[i]) / windowSize;

        dvec2 norm_pos;

        norm_pos.x = mix(-1.0, 1.0, pos.x);
	    norm_pos.y = mix(-1.0, 1.0, pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // one more for mouse
    if (lastVertexMouse) {
        
        dvec2 norm_mouseScreenSpace;
        norm_mouseScreenSpace.x = mix(-1.0, 1.0, norm_mouse.x);
        norm_mouseScreenSpace.y = mix(-1.0, 1.0, norm_mouse.y);
        gl_Position = vec4(float(norm_mouseScreenSpace.x), float(norm_mouseScreenSpace.y), 0.0, 1.0);
        EmitVertex();
    } else {
        dvec2 pos = (polygonCenter + vertices[0]) / windowSize;

        dvec2 norm_pos;

        norm_pos.x = mix(-1.0, 1.0, pos.x);
	    norm_pos.y = mix(-1.0, 1.0, pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    EndPrimitive();
}