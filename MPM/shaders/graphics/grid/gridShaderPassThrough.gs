#version 450 core

in vec4 vs_nodeColor[];
in int gridNodeI[];
in int gridNodeJ[];

out vec4 gs_nodeColor;

layout (points) in;
layout (line_strip, max_vertices = 5) out;

uniform double zoomFactor;
uniform dvec2 zoomPoint;

//uniform int selectedVector = 1;

/*** HEADER ***/

void main() {

    // Goal, to make a cross for each grid point (so when combined it looks like a grid)

    //int maxDim = max(GRID_SIZE_X, GRID_SIZE_Y);
    //dvec2 grid_vec = dvec2(maxDim, maxDim);

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
    // get normalized zoomPoint;
    dvec2 norm_zoomPoint = zoomPoint / grid_vec;


    int nodei = gridNodeI[0];
    int nodej = gridNodeJ[0];
    gridNode node;
    GetNode(gridNodeI[0], gridNodeJ[0], node);

    dvec2 gridPos = dvec2(nodei, nodej);

    // norm_pos mapped to (0, 1)
    dvec2 norm_middle_pos = gridPos / grid_vec;
    
    norm_middle_pos -= norm_zoomPoint;
    norm_middle_pos *= zoomFactor;
    norm_middle_pos += norm_zoomPoint;

    // map to opengl window space (-1.0, 1.0)
    dvec2 ws_middle_pos;
	ws_middle_pos.x = mix(-1.0, 1.0, norm_middle_pos.x);
	ws_middle_pos.y = mix(-1.0, 1.0, norm_middle_pos.y);
    vec4 ws_middle_posF = vec4(float(ws_middle_pos.x), float(ws_middle_pos.y), 0.0, 1.0);


    // UPPER
    dvec2 norm_up_pos = (gridPos + dvec2(0, h)) / grid_vec;
    norm_up_pos -= norm_zoomPoint;
    norm_up_pos *= zoomFactor;
    norm_up_pos += norm_zoomPoint;
    dvec2 ws_up_pos;
    ws_up_pos.x = mix(-1.0, 1.0, norm_up_pos.x);
	ws_up_pos.y = mix(-1.0, 1.0, norm_up_pos.y);
    vec4 ws_up_posF = vec4(float(ws_up_pos.x), float(ws_up_pos.y), 0.0, 1.0);

    // RIGHT
    dvec2 norm_right_pos = (gridPos + dvec2(h, 0)) / grid_vec;
    norm_right_pos -= norm_zoomPoint;
    norm_right_pos *= zoomFactor;
    norm_right_pos += norm_zoomPoint;
    dvec2 ws_right_pos;
    ws_right_pos.x = mix(-1.0, 1.0, norm_right_pos.x);
	ws_right_pos.y = mix(-1.0, 1.0, norm_right_pos.y);
    vec4 ws_right_posF = vec4(float(ws_right_pos.x), float(ws_right_pos.y), 0.0, 1.0);

    // DOWN
    dvec2 norm_down_pos = (gridPos + dvec2(0, -h)) / grid_vec;
    norm_down_pos -= norm_zoomPoint;
    norm_down_pos *= zoomFactor;
    norm_down_pos += norm_zoomPoint;
    dvec2 ws_down_pos;
    ws_down_pos.x = mix(-1.0, 1.0, norm_down_pos.x);
	ws_down_pos.y = mix(-1.0, 1.0, norm_down_pos.y);
    vec4 ws_down_posF = vec4(float(ws_down_pos.x), float(ws_down_pos.y), 0.0, 1.0);

    // LEFT
    dvec2 norm_left_pos = (gridPos + dvec2(-h, 0)) / grid_vec;
    norm_left_pos -= norm_zoomPoint;
    norm_left_pos *= zoomFactor;
    norm_left_pos += norm_zoomPoint;
    dvec2 ws_left_pos;
    ws_left_pos.x = mix(-1.0, 1.0, norm_left_pos.x);
	ws_left_pos.y = mix(-1.0, 1.0, norm_left_pos.y);
    vec4 ws_left_posF = vec4(float(ws_left_pos.x), float(ws_left_pos.y), 0.0, 1.0);


    gl_Position = ws_left_posF;
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();

    gl_Position = ws_right_posF;
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();

    gl_Position = ws_middle_posF;
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();

    gl_Position = ws_up_posF;
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();

    gl_Position = ws_down_posF;
    gl_PointSize = gl_in[0].gl_PointSize;
    gs_nodeColor = vs_nodeColor[0];
    EmitVertex();
    
    EndPrimitive();
}