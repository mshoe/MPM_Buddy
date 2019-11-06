#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 256) out;
// 36320 is max

uniform dvec4 mouse;

uniform double radius;

uniform dvec2 zoomPoint;
uniform double zoomFactor;
/*** HEADER ***/


void main() {

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    dvec2 center = mouse.xy;
    dvec2 norm_center = center / grid_vec;
    norm_center.x = mix(-1.0, 1.0, norm_center.x);
    norm_center.y = mix(-1.0, 1.0, norm_center.y);



    float PI = 3.14159;
    int N = 20;
    for (int i = 0; i < N + 1; i++) {

        float sinusoidalValue = 2.0 * float(i) / float(N) * PI;
        dvec2 circle_point = dvec2(double(cos(sinusoidalValue)), double(sin(sinusoidalValue)));

        dvec2 pos = (center + radius * circle_point);

        
        pos -= zoomPoint;
        pos *= zoomFactor;
        pos += zoomPoint;

        dvec2 norm_pos = pos / grid_vec;;

        norm_pos.x = mix(-1.0, 1.0, norm_pos.x);
	    norm_pos.y = mix(-1.0, 1.0, norm_pos.y);

        gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
        EmitVertex();
    }

    // one more for mouse


    // gl_Position = vec4(0.7, 0.7, 0.0, 1.0);
    // EmitVertex();

    // gl_Position = vec4(0.9, 0.9, 0.0, 1.0);
    // EmitVertex();

    EndPrimitive();
}