#version 450 core

layout (location = 0) in vec2 vert;

void main() {

    // float new_xpos;
    // float new_ypos;

    // // map vertex to (-1.0, 1.0)

    // if (gl_VertexID == 0) { // top right
    //     //new_xpos = 1.0;
    //     //new_ypos = 1.0;
    //     new_xpos = float((iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0);
    //     new_ypos = float((iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0);
    // } else if (gl_VertexID == 1) { // bottom right
    //     //new_xpos = 1.0;
    //     //new_ypos = -1.0;
    //     new_xpos = float((iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0);
    //     new_ypos = float((iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0);
    // } else if (gl_VertexID == 2) { // bottom left
    //     //new_xpos = -1.0;
    //     //new_ypos = -1.0;
    //     new_xpos = float((iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0);
    //     new_ypos = float((iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0);
    // } else if (gl_VertexID == 3) { // top left
    //     //new_xpos = -1.0;
    //     //new_ypos = 1.0;
    //     new_xpos = float((iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0);
    //     new_ypos = float((iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0);
    // }

    // gl_Position = vec4(new_xpos, new_ypos, 0.0, 1.0);
    gl_Position = vec4(vert, 0.0, 1.0);
}