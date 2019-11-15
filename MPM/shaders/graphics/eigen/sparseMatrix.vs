#version 450 core

out flat vec4 vs_tripletColor;

struct eigenTriplet {
    ivec2 index; // base alignment = 8, offset = 0
    double val; // base alignment = 8, offset = 8
};

layout (std430, binding = 9) buffer EigenTriplets {
    eigenTriplet triplets[];
};


uniform int spMatRows;

uniform float left_edge = -0.9;
uniform float bot_edge = -0.9;
uniform float right_edge = 0.9;
uniform float top_edge = 0.9;




void main() {
    int N = triplets.length();
    if (gl_VertexID > N) {
        return;
    }

    ivec2 index = triplets[gl_VertexID].index;
    double val = triplets[gl_VertexID].val;


    vec2 findex = vec2(float(index.x), float(index.y));
    vec2 findex_normalized = findex / vec2(float(spMatRows), float(spMatRows));
    
    // flip y around cuz this is glsl
    findex_normalized.y = 1.0 - findex_normalized.y;

    // map sparse matrix to a square
    findex_normalized.x = mix(left_edge, right_edge, findex_normalized.x);
    findex_normalized.y = mix(bot_edge, top_edge, findex_normalized.y);



    
    


    if (isnan(val)) {
        vs_tripletColor = vec4(0.0, 0.0, 1.0, 1.0); // blue
    }
    else if (val > 0.0) {
        vs_tripletColor = vec4(0.0, 1.0, 0.0, 1.0); // green
    }
    else if (val == 0.0) {
        vs_tripletColor = vec4(1.0, 1.0, 1.0, 1.0); // white
    }
    else if (val < 0.0) {
        vs_tripletColor = vec4(1.0, 0.0, 0.0, 1.0); // red
    }

    gl_Position = vec4(findex_normalized, 0.0, 1.0);
    gl_PointSize = 2.0;
}