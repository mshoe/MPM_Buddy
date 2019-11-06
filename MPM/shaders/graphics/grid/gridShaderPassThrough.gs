#version 450 core

in vec4 vs_nodeColor[];
in int gridNodeI[];
in int gridNodeJ[];

out vec4 gs_nodeColor;

layout (points) in;
layout (points, max_vertices = 1) out;

//uniform int selectedVector = 1;

/*** HEADER ***/

void main() {
    gl_Position = gl_in[0].gl_Position; // + vec4(-0.1, 0.0, 0.0, 0.0);
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();
    
    EndPrimitive();
}