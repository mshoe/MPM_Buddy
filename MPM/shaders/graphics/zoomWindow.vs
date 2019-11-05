#version 450 core

//uniform double zoomFactor;
uniform dvec2 zoomPoint; // IN GRID SPACE
//uniform dvec2 dimensions; // IN GRID SPACE
//uniform double zoomRadius = 20.0;
uniform double zoomFactor;

// CONVERT WINDOW GIVEN IN GRID SPACE TO WINDOW IN SCREEN SPACE

/*** HEADER ***/

// IN MAIN SCREEN SPACE
void main() {

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

    // then determine the point in MPM screen space
    double new_xpos;
    double new_ypos;

    if (gl_VertexID == 0) { // top right
        //new_xpos = 1.0;
        //new_ypos = 1.0;
        new_xpos = mix(-1.0, 1.0, grid_top_right.x);
        new_ypos = mix(-1.0, 1.0, grid_top_right.y);
    } else if (gl_VertexID == 1) { // bottom right
        //new_xpos = 1.0;
        //new_ypos = -1.0;
        new_xpos = mix(-1.0, 1.0, grid_bot_right.x);
        new_ypos = mix(-1.0, 1.0, grid_bot_right.y);
    } else if (gl_VertexID == 2) { // bottom left
        //new_xpos = -1.0;
        //new_ypos = -1.0;
        new_xpos = mix(-1.0, 1.0, grid_bot_left.x);
        new_ypos = mix(-1.0, 1.0, grid_bot_left.y);
    } else if (gl_VertexID == 3) { // top left
        //new_xpos = -1.0;
        //new_ypos = 1.0;
        new_xpos = mix(-1.0, 1.0, grid_top_left.x);
        new_ypos = mix(-1.0, 1.0, grid_top_left.y);
    }

    gl_Position = vec4(new_xpos, new_ypos, 0.0, 1.0);
}