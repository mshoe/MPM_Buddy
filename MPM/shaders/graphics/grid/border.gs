#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 5) out;

/*** HEADER ***/

uniform dvec2 zoomPoint;
uniform double zoomFactor;

void zoom(inout vec2 normPos, in vec2 normZoomPoint) {
    normPos -= normZoomPoint;
    normPos *= float(zoomFactor);
    normPos += normZoomPoint;
}

void main() {
    // Only calling this shader when numVertices > 1, managed in c++ code


    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    vec2 bot_left_grid_corner = vec2(-1.0, -1.0);
    vec2 top_left_grid_corner = vec2(-1.0, -1.0 + 2.0 * float(CHUNKS_Y) / 4.0);
    vec2 top_right_grid_corner = vec2(-1.0 + 2.0 * float(CHUNKS_X) / 4.0, -1.0 + 2.0 * float(CHUNKS_Y) / 4.0);
    vec2 bot_right_grid_corner = vec2(-1.0 + 2.0 * float(CHUNKS_X) / 4.0, -1.0);

    vec2 normZoomPoint = vec2(float((zoomPoint / grid_vec).x), float((zoomPoint / grid_vec).y));
    normZoomPoint.x = mix(-1.0, 1.0, normZoomPoint.x);
    normZoomPoint.y = mix(-1.0, 1.0, normZoomPoint.y);

    zoom(bot_left_grid_corner, normZoomPoint);
    zoom(top_left_grid_corner, normZoomPoint);
    zoom(top_right_grid_corner, normZoomPoint);
    zoom(bot_right_grid_corner, normZoomPoint);

    gl_Position = vec4(bot_left_grid_corner, 0.0, 1.0);
    EmitVertex();

    gl_Position = vec4(top_left_grid_corner, 0.0, 1.0);
    EmitVertex();

    
    gl_Position = vec4(top_right_grid_corner, 0.0, 1.0);
    EmitVertex();

    
    gl_Position = vec4(bot_right_grid_corner, 0.0, 1.0);
    EmitVertex();

    gl_Position = vec4(bot_left_grid_corner, 0.0, 1.0);
    EmitVertex();

    // gl_Position = vec4(0.7, 0.7, 0.0, 1.0);
    // EmitVertex();

    // gl_Position = vec4(0.9, 0.9, 0.0, 1.0);
    // EmitVertex();

    EndPrimitive();
} 