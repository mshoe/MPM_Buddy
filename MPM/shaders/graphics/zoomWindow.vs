#version 450 core

//uniform double zoomFactor;
uniform dvec2 zoomPoint; // IN GRID SPACE
//uniform dvec2 dimensions; // IN GRID SPACE
//uniform double zoomRadius = 20.0;
uniform double zoomFactor;

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)

// CONVERT WINDOW GIVEN IN GRID SPACE TO WINDOW IN SCREEN SPACE

/*** HEADER ***/

// IN MAIN SCREEN SPACE
void main() {

    //double zoomRadius = double(GRID_SIZE_X) / 2.0 / zoomFactor;

    // screen space borders of the MPM grid
    double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // grid space borders
    dvec2 grid_top_right = dvec2(1.0, 1.0);
    dvec2 grid_bot_right = dvec2(1.0, 0.0);
    dvec2 grid_bot_left = dvec2(0.0, 0.0);
    dvec2 grid_top_left = dvec2(0.0, 1.0);

    // determine the zoom point in normalized grid space
    dvec2 norm_zoomPoint = zoomPoint / grid_vec;

    // transform the grid space borders
    grid_top_right -= norm_zoomPoint;
    grid_top_right *= 1.0 / zoomFactor;
    grid_top_right += norm_zoomPoint;

    grid_bot_right -= norm_zoomPoint;
    grid_bot_right *= 1.0 / zoomFactor;
    grid_bot_right += norm_zoomPoint;

    grid_bot_left -= norm_zoomPoint;
    grid_bot_left *= 1.0 / zoomFactor;
    grid_bot_left += norm_zoomPoint;

    grid_top_left -= norm_zoomPoint;
    grid_top_left *= 1.0 / zoomFactor;
    grid_top_left += norm_zoomPoint;

    // then convert to screen space

    // double norm_zoomPointLeftBorder = (zoomPoint.x - zoomRadius) / grid_vec.x;
    // double norm_zoomPointRightBorder = (zoomPoint.x + zoomRadius) / grid_vec.x;
    // double norm_zoomPointBottomBorder = (zoomPoint.y - zoomRadius) / grid_vec.y;
    // double norm_zoomPointTopBorder = (zoomPoint.y + zoomRadius) / grid_vec.y;

    // then determine the point in MPM screen space
    double new_xpos;
    double new_ypos;

    if (gl_VertexID == 0) { // top right
        //new_xpos = 1.0;
        //new_ypos = 1.0;
        new_xpos = mix(left_border, right_border, grid_top_right.x);
        new_ypos = mix(bottom_border, top_border, grid_top_right.y);
    } else if (gl_VertexID == 1) { // bottom right
        //new_xpos = 1.0;
        //new_ypos = -1.0;
        new_xpos = mix(left_border, right_border, grid_bot_right.x);
        new_ypos = mix(bottom_border, top_border, grid_bot_right.y);
    } else if (gl_VertexID == 2) { // bottom left
        //new_xpos = -1.0;
        //new_ypos = -1.0;
        new_xpos = mix(left_border, right_border, grid_bot_left.x);
        new_ypos = mix(bottom_border, top_border, grid_bot_left.y);
    } else if (gl_VertexID == 3) { // top left
        //new_xpos = -1.0;
        //new_ypos = 1.0;
        new_xpos = mix(left_border, right_border, grid_top_left.x);
        new_ypos = mix(bottom_border, top_border, grid_top_left.y);
    }

    gl_Position = vec4(new_xpos, new_ypos, 0.0, 1.0);
}