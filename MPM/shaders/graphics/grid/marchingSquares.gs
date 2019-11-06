#version 450 core

in vec4 vs_nodeColor[];
in int gridNodeI[];
in int gridNodeJ[];

out vec4 gs_nodeColor;

layout (points) in;
layout (triangle_strip, max_vertices = 6) out;

uniform double zoomFactor;
uniform dvec2 zoomPoint;
uniform double minMass = 0.0;
uniform double isoMass;
uniform vec4 mscolor;

/*** HEADER ***/

void MarchingSquaresSwitch( vec2 v00, vec2 v01, vec2 v10, vec2 v11, 
                            float m00, float m01, float m10, float m11, 
                            vec4 nodeColor, uint mscase) {


    // These 4 "midpoints" get will change based on where the isoMass is found.
    // isoMass is guessed via linear interpolation
    vec2 v00_10 = (v00 + v10) / 2.0;
    vec2 v00_01 = (v00 + v01) / 2.0;

    vec2 v10_11 = (v10 + v11) / 2.0;
    vec2 v01_11 = (v01 + v11) / 2.0;

    float isoMassf = float(isoMass);
    float minMassf = float(minMass);
    
    if (m10 > m00) {
        v00_10.y = mix(v00.y, v10.y, (isoMassf - m00) / (m10 - m00));
    } else if (m10 < m00) {
        v00_10.y = mix(v10.y, v00.y, (isoMassf - m10) / (m00 - m10));
    }

    if (m01 > m00) {
        v00_01.x = mix(v00.x, v01.x, (isoMassf - m00) / (m01 - m00));
    } else if (m01 < m00) {
        v00_01.x = mix(v01.x, v00.x, (isoMassf - m01) / (m00 - m01));
    }
    
    if (m10 > m11) {
        v10_11.x = mix(v11.x, v10.x, (isoMassf - m11) / (m10 - m11));
    } else if (m10 < m11) {
        v10_11.x = mix(v10.x, v11.x, (isoMassf - m10) / (m11 - m10));
    }

    if (m01 > m11) {
        v01_11.y = mix(v11.y, v01.y, (isoMassf - m11) / (m01 - m11));
    } else if (m01 < m11) {
        v01_11.y = mix(v01.y, v11.y, (isoMassf - m01) / (m11 - m01));
    }
    

    //vec4 test = vec4(v00, 0.0, 1.0);

    switch (mscase) {
    case 0:
        break;
    case 1:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    case 2:
        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    case 3:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        
        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    case 4:
        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    case 5:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    case 6:
        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 7:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 8:
        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 9:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    
    case 10:
        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 11:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    
    case 12:
        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 13:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01_11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 14:
        gl_Position = vec4(v00_10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v00_01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;

    case 15:
        gl_Position = vec4(v00, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v10, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();

        gl_Position = vec4(v01, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        
        gl_Position = vec4(v11, 0.0, 1.0);
        gs_nodeColor = nodeColor;
        EmitVertex();
        break;
    default:
        break;
    }
}

void main() {

    // Marching squares marches through the squares of the grid.
    // A square is a 2x2 grid.
    // If there are n x n grid nodes, then there are (n - 1) x (n - 1) squares

    int nodei = gridNodeI[0];
    int nodej = gridNodeJ[0];
    if (nodei >= GRID_SIZE_X || nodej >= GRID_SIZE_Y) { // ??? why is it not going out of bounds
        return;
    }

    // CONFUSING i -> x direction, j -> y direction.
    // confusing because index ordered [i][j] like (row, column), where changing rows would change y

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    double mass00 = nodes[nodei][nodej].m;
    double mass01 = nodes[nodei+1][nodej].m;
    double mass10 = nodes[nodei][nodej+1].m;
    double mass11 = nodes[nodei+1][nodej+1].m;

    // map square of grid nodes to (0.0, 1.0)
    dvec2 v00 = dvec2(double(nodei), double(nodej)) / grid_vec;
    dvec2 v01 = dvec2(double(nodei+1), double(nodej)) / grid_vec;
    dvec2 v10 = dvec2(double(nodei), double(nodej+1)) / grid_vec;
    dvec2 v11 = dvec2(double(nodei+1), double(nodej+1)) / grid_vec;

    // then map them to (-1.0, 1.0) based on the borders
    v00.x = mix(-1.0, 1.0, v00.x);
    v00.y = mix(-1.0, 1.0, v00.y);
    v01.x = mix(-1.0, 1.0, v01.x);
    v01.y = mix(-1.0, 1.0, v01.y);
    v10.x = mix(-1.0, 1.0, v10.x);
    v10.y = mix(-1.0, 1.0, v10.y);
    v11.x = mix(-1.0, 1.0, v11.x);
    v11.y = mix(-1.0, 1.0, v11.y);
    
    bool iso00 = mass00 >= isoMass;
    bool iso01 = mass01 >= isoMass;
    bool iso10 = mass10 >= isoMass;
    bool iso11 = mass11 >= isoMass;

    uint mscase = 0;
    if (iso00) {
        mscase = mscase | 1;
    }
    if (iso01) {
        mscase = mscase | 2;
    }
    if (iso11) { // counter clockwise ordering
        mscase = mscase | 4;
    }
    if (iso10) {
        mscase = mscase | 8;
    }

    vec4 nodeColor = mscolor;

    MarchingSquaresSwitch(  vec2(float(v00.x), float(v00.y)), 
                            vec2(float(v01.x), float(v01.y)), 
                            vec2(float(v10.x), float(v10.y)), 
                            vec2(float(v11.x), float(v11.y)),
                            float(mass00), float(mass01), float(mass10), float(mass11),
                            nodeColor, mscase);

    EndPrimitive();
}