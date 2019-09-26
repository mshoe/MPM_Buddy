#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 4) out;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)

//uniform dvec2 zoomPoint;
//uniform double zoomFactor;

/*** HEADER ***/


void main() {
    // Only calling this shader when numVertices > 1, managed in c++ code

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

    
    dvec2 bot_left_grid_corner = dvec2(left_border, bottom_border);
    gl_Position = vec4(float(bot_left_grid_corner.x), float(bot_left_grid_corner.y), 0.0, 1.0);
    EmitVertex();

    dvec2 top_left_grid_corner = dvec2(left_border, bottom_border + (top_border - bottom_border) * double(CHUNKS_Y) / 4.0);
    gl_Position = vec4(float(top_left_grid_corner.x), float(top_left_grid_corner.y), 0.0, 1.0);
    EmitVertex();

    dvec2 top_right_grid_corner = dvec2(left_border + (right_border - left_border) * double(CHUNKS_X) / 4.0, bottom_border + (top_border - bottom_border) * double(CHUNKS_Y) / 4.0);
    gl_Position = vec4(float(top_right_grid_corner.x), float(top_right_grid_corner.y), 0.0, 1.0);
    EmitVertex();

    dvec2 bot_right_grid_corner = dvec2(left_border + (right_border - left_border) * double(CHUNKS_X) / 4.0, bottom_border);
    gl_Position = vec4(float(bot_right_grid_corner.x), float(bot_right_grid_corner.y), 0.0, 1.0);
    EmitVertex();

    

    // gl_Position = vec4(0.7, 0.7, 0.0, 1.0);
    // EmitVertex();

    // gl_Position = vec4(0.9, 0.9, 0.0, 1.0);
    // EmitVertex();

    EndPrimitive();
}