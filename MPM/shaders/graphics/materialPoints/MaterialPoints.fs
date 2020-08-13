#version 450 core

flat in uint gs_pointID;

uniform vec4 pointCloudColor = vec4(1.0, 1.0, 0.0, 1.0);

out vec4 fragColor;

/*** HEADER ***/

void main() {
    dvec4 strain = points[gs_pointID].strain;
    dvec4 stress = points[gs_pointID].stress;
    dvec4 rgba = points[gs_pointID].rgba;
    dvec2 pos = points[gs_pointID].x;
    dmat2 P = points[gs_pointID].P;
    dmat2 B = points[gs_pointID].B;

    //fragColor = vec4(float(strain.x), float(strain.y), float(strain.z), float(strain.w));
    //fragColor = vec4(float(P[0][0]), float(P[0][1]), float(P[1][0]), 1.0);
    //fragColor = vec4(float(points[gs_pointID].energy), 0.0, 0.0, 1.0);
    fragColor = vec4(rgba.x, rgba.y, rgba.z, rgba.w);
}