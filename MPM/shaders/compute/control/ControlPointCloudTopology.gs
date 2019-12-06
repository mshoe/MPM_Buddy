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

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 center = points[vs_pointID[0]].x.xy;

    center -= zoomPoint;
    center *= zoomFactor;
    center += zoomPoint;

    dvec2 norm_center = center / grid_vec;
    norm_center.x = mix(-1.0, 1.0, norm_center.x);
    norm_center.y = mix(-1.0, 1.0, norm_center.y);
    

    dmat2 F = points[vs_pointID[0]].F;
    dmat2 dFc = points[vs_pointID[0]].dFc;

    int botLeftNode_i = int(floor(center.x)) - 1;
	int botLeftNode_j = int(floor(center.y)) - 1;


    for (int i = 0; i <= 3; ++i) {
		for (int j = 0; j <= 3; ++j) {
			int curNode_i = botLeftNode_i + i;
			int curNode_j = botLeftNode_j + j;

			//dvec2 dpg = dvec2(curNode_i, curNode_j) - pos;
			//double dx = dpg.x;
			//double dy = dpg.y;
			//double wpg = BSpline(dx) * BSpline(dy);



            gl_Position = vec4(norm_center.x, norm_center.y, 0.0, 1.0);
            EmitVertex();


            dvec2 curNodePos = dvec2(curNode_i, curNode_j);
            
            curNodePos -= zoomPoint;
            curNodePos *= zoomFactor;
            curNodePos += zoomPoint;

            dvec2 norm_curNodPos = curNodePos / grid_vec;

            gl_Position = vec4(float(curNode_i), float(curNode_j), 0.0, 1.0);
            EmitVertex();
        }
    }
    
    EndPrimitive();
}