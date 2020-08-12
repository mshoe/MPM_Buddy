#version 450 core

in uint vs_pointID[];

layout (points) in;
layout (line_strip, max_vertices = 256) out;

uniform double zoomFactor;
uniform dvec2 zoomPoint;
uniform double pointRadius = 1.0;

uniform bool renderDeformationGradients;
uniform bool renderControlDeformationGradients;

/*** HEADER ***/

#define PI 3.1415926535897932384626433832795

void main() {
    //gl_Position = gl_in[0].gl_Position; // + vec4(-0.1, 0.0, 0.0, 0.0);
    //gl_PointSize = gl_in[0].gl_PointSize;

    dvec2 grid_vec = vec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 center = points[vs_pointID[0]].x.xy;

    dmat2 F = points[vs_pointID[0]].F;
    dmat2 dFc = points[vs_pointID[0]].dFc;

    int N = 20;
    for (int i = 0; i < N + 1; i++) {

        float sinusoidalValue = 2.0 * float(i) / float(N) * PI;
        dvec2 circle_point = dvec2(double(cos(sinusoidalValue)), double(sin(sinusoidalValue)));

        if (renderDeformationGradients) {
            circle_point = (F + dFc) * circle_point;
        } else if (renderControlDeformationGradients) {
            circle_point = dFc * circle_point;
        }

        dvec2 pos = (center + pointRadius * circle_point);

        
        pos -= zoomPoint;
        pos *= zoomFactor;
        pos += zoomPoint;

        dvec2 norm_pos = pos / grid_vec;;

        norm_pos.x = mix(-1.0, 1.0, norm_pos.x);
	    norm_pos.y = mix(-1.0, 1.0, norm_pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }
    
    EndPrimitive();
}