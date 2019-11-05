#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 100) out;

uniform dvec2 polygonCenter;
uniform dvec2 windowSize; // w.r.t. to grid_vec = (128, 128)
layout (std430, binding = 3) buffer polygonVertices {
	dvec2 vertices[];
};

uniform dvec4 mouse; // this should be the mouse coords w.r.t. this window (managed in C++ code)

uniform bool lastVertexMouse = false;

void main() {
    int numVertices = vertices.length();
    // Only calling this shader when numVertices > 1, managed in c++ code

    for (int i = 0; i < numVertices; i++) {
        dvec2 pos = (polygonCenter + vertices[i]) / windowSize;

        gl_Position = vec4(float(pos.x), float(pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // one more for mouse
    if (lastVertexMouse) {
        gl_Position = vec4(float(mouse.x), float(mouse.y), 0.0, 1.0);
        EmitVertex();
    } else {
        dvec2 pos = (polygonCenter + vertices[0]) / windowSize;

        gl_Position = vec4(float(pos.x), float(pos.y), 0.0, 1.0);
        EmitVertex();
    }

    EndPrimitive();
}