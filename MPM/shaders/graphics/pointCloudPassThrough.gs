#version 450 core

in vec4 vs_stressColor[];
in uint pointID[];
out vec4 gs_stressColor;

layout (points) in;
layout (points, max_vertices = 1) out;

/*** HEADER ***/

void main() {
    gl_Position = gl_in[0].gl_Position; // + vec4(-0.1, 0.0, 0.0, 0.0);
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_stressColor = vs_stressColor[0];

    EmitVertex();
    
    EndPrimitive();
}